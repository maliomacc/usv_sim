#!/usr/bin/env python3
"""
model_test_live.py — STI USV Canlı Model Test Aracı
=====================================================
ZED kamerasından görüntü alır, YOLO inference çalıştırır,
OpenCV penceresiyle canlı sonuçları gösterir.

YOLO Sınıfları (son.engine / son.pt):
  0=Black  1=Green  2=Orange  3=Red  4=Yellow

Tuşlar:
  q         → çıkış
  s         → ekran görüntüsü kaydet (screenshot_XXXX.jpg)
  0-4       → sadece o sınıfı göster  (0=Black 1=Green 2=Orange 3=Red 4=Yellow)
  a         → tüm sınıfları göster
  +/-       → confidence eşiğini artır/azalt (±0.05)
  d         → derinlik değerlerini aç/kapat
  SPACE     → dondur/devam et

Kullanım:
  python3 model_test_live.py
  python3 model_test_live.py --model son.pt
  python3 model_test_live.py --conf 0.5 --filter 3
"""

import argparse
import os
import time
import sys

import cv2
import numpy as np

# ── Argümanlar ────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--model',  default=None,  help='Model yolu (.pt veya .engine)')
parser.add_argument('--conf',   type=float, default=0.35, help='Confidence eşiği')
parser.add_argument('--filter', type=int,   default=-1,   help='Sadece bu sınıf (-1=hepsi)')
parser.add_argument('--no-depth', action='store_true',    help='Derinlik gösterme')
args = parser.parse_args()

# ── Model yolları (öncelik sırası) ────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR  = os.path.join(os.path.dirname(SCRIPT_DIR), 'models')

MODEL_CANDIDATES = [
    args.model,
    os.path.join(MODEL_DIR, 'son.engine'),
    os.path.join(MODEL_DIR, 'son.pt'),
    os.path.join(MODEL_DIR, '252epoch.pt'),
]

# ── YOLO Sınıf tanımları (son.engine / son.pt) ────────────────────────────────
CLS_NAMES  = {0: 'Black', 1: 'Green', 2: 'Orange', 3: 'Red', 4: 'Yellow'}
CLS_COLORS = {   # BGR
    0: (50,  50,  50),    # Black  → koyu gri kutu
    1: (0,   200, 50),    # Green  → yeşil
    2: (0,   140, 255),   # Orange → turuncu (BGR)
    3: (0,   0,   220),   # Red    → kırmızı
    4: (0,   220, 220),   # Yellow → sarı
}


def load_model():
    from ultralytics import YOLO
    for path in MODEL_CANDIDATES:
        if path is None:
            continue
        if not os.path.exists(path):
            continue
        print(f'[MODEL] Yükleniyor: {path}')
        try:
            m = YOLO(path, task='detect')
            # Isınma (ilk inference yavaş olur)
            dummy = np.zeros((180, 320, 3), dtype=np.uint8)
            m.predict(dummy, device='0', verbose=False, conf=0.1)
            print(f'[MODEL] ✓ Hazır | sınıflar: {m.names}')
            return m, path
        except Exception as e:
            print(f'[MODEL] {os.path.basename(path)} yüklenemedi: {e}')
    raise RuntimeError('Hiçbir model yüklenemedi!')


def open_zed():
    import pyzed.sl as sl
    cam    = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution    = sl.RESOLUTION.HD720   # 1280×720
    params.camera_fps           = 30
    params.depth_mode           = sl.DEPTH_MODE.PERFORMANCE
    params.coordinate_units     = sl.UNIT.METER
    params.depth_minimum_distance = 0.3
    status = cam.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f'ZED açılamadı: {status}')
    info = cam.get_camera_information()
    print(f'[ZED] ✓ {info.camera_model} | SN:{info.serial_number} | '
          f'{info.camera_configuration.resolution.width}×'
          f'{info.camera_configuration.resolution.height}')
    return cam, sl


def draw_detections(frame, dets, conf_thresh, cls_filter, show_depth):
    h, w = frame.shape[:2]
    counts = {k: 0 for k in CLS_NAMES}

    for d in dets:
        cls_id = d['cls']
        conf   = d['conf']
        if conf < conf_thresh:
            continue
        if cls_filter >= 0 and cls_id != cls_filter:
            continue

        counts[cls_id] = counts.get(cls_id, 0) + 1
        col  = CLS_COLORS.get(cls_id, (200, 200, 200))
        name = CLS_NAMES.get(cls_id, str(cls_id))

        x1, y1, x2, y2 = d['x1'], d['y1'], d['x2'], d['y2']
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)

        label = f'{name} {conf:.2f}'
        if show_depth and d.get('depth') is not None:
            label += f' {d["depth"]:.1f}m'

        (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        ty = max(y1 - 6, lh + 4)
        cv2.rectangle(frame, (x1, ty - lh - 4), (x1 + lw + 4, ty + 2), col, -1)
        cv2.putText(frame, label, (x1 + 2, ty - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

        # Merkez nokta
        cx, cy = int(d['cx']), int(d['cy'])
        cv2.drawMarker(frame, (cx, cy), col, cv2.MARKER_CROSS, 12, 2)

    return counts


def draw_hud(frame, fps, conf_thresh, cls_filter, counts, model_name, paused):
    h, w = frame.shape[:2]

    # Üst bilgi şeridi
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 36), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)

    fps_col  = (0, 220, 0) if fps >= 15 else (0, 100, 255) if fps >= 8 else (0, 0, 220)
    filt_str = CLS_NAMES.get(cls_filter, 'TÜMÜ') if cls_filter >= 0 else 'TÜMÜ'
    status   = '❚❚ DONDURULDU' if paused else '▶ CANLI'

    cv2.putText(frame,
        f'{status}  |  {os.path.basename(model_name)}  |  conf≥{conf_thresh:.2f}  |  filtre:{filt_str}',
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(frame, f'{fps:.0f} FPS', (w - 80, 22),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, fps_col, 1, cv2.LINE_AA)

    # Alt tespit sayacı
    y_bot = h - 8
    x     = 8
    for cls_id, cnt in sorted(counts.items()):
        if cnt == 0 and cls_filter >= 0 and cls_id != cls_filter:
            continue
        col  = CLS_COLORS[cls_id]
        text = f'{CLS_NAMES[cls_id]}:{cnt}'
        col_display = col if cnt > 0 else (80, 80, 80)
        cv2.putText(frame, text, (x, y_bot),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, col_display, 1, cv2.LINE_AA)
        x += len(text) * 9 + 12

    # Tuş yardımı (sağ alt)
    help_lines = ['q:çıkış', 's:kaydet', '0-4:filtre', 'a:hepsi', '+/-:conf', 'd:derinlik', 'SPC:dondur']
    for i, line in enumerate(reversed(help_lines)):
        cv2.putText(frame, line, (w - 130, h - 10 - i * 16),
            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 120, 120), 1, cv2.LINE_AA)


def sample_depth(depth_np, cx, cy, win=4):
    if depth_np is None:
        return None
    h, w = depth_np.shape[:2]
    y0 = max(0, cy - win);  y1 = min(h, cy + win + 1)
    x0 = max(0, cx - win);  x1 = min(w, cx + win + 1)
    patch = depth_np[y0:y1, x0:x1]
    valid = patch[np.isfinite(patch) & (patch > 0.3) & (patch < 30.0)]
    return float(np.median(valid)) if valid.size > 0 else None


def main():
    # Model yükle
    model, model_path = load_model()

    # ZED aç
    cam, sl = open_zed()
    image_zed = sl.Mat()
    depth_zed = sl.Mat()

    conf_thresh = args.conf
    cls_filter  = args.filter
    show_depth  = not args.no_depth
    paused      = False
    frozen_frame = None
    screenshot_n = 0

    print()
    print('╔═══════════════════════════════════════════════╗')
    print('║  STI USV — Canlı Model Test                   ║')
    print('║  q:çıkış  s:kaydet  0-4:filtre  a:hepsi       ║')
    print('║  +/-:conf eşiği     d:derinlik  SPC:dondur     ║')
    print('╚═══════════════════════════════════════════════╝')

    cv2.namedWindow('STI USV — Model Test (q=cikis)', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('STI USV — Model Test (q=cikis)', 1280, 720)

    fps_buf  = []
    t_prev   = time.time()

    while True:
        # ── Görüntü al ────────────────────────────────────────────────────────
        if not paused:
            if cam.grab() != sl.ERROR_CODE.SUCCESS:
                print('[HATA] Kare alınamadı')
                time.sleep(0.01)
                continue

            cam.retrieve_image(image_zed, sl.VIEW.LEFT)
            bgr = cv2.cvtColor(image_zed.get_data(), cv2.COLOR_BGRA2BGR)

            depth_np = None
            if show_depth:
                cam.retrieve_measure(depth_zed, sl.MEASURE.DEPTH)
                depth_np = depth_zed.get_data().copy()

            # ── YOLO inference ────────────────────────────────────────────────
            results = model.predict(bgr, device='0', verbose=False, conf=0.05)

            dets = []
            if results and results[0].boxes is not None:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].cpu().numpy()]
                    cls_id = int(box.cls[0].cpu().item())
                    conf   = float(box.conf[0].cpu().item())
                    cx     = (x1 + x2) // 2
                    cy     = (y1 + y2) // 2
                    depth_val = sample_depth(depth_np, cx, cy) if show_depth else None
                    dets.append({
                        'cls': cls_id, 'conf': conf,
                        'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                        'cx': cx, 'cy': cy, 'depth': depth_val,
                    })

            frozen_frame = (bgr.copy(), dets, depth_np)

        else:
            bgr, dets, depth_np = frozen_frame

        # ── Görselleştir ──────────────────────────────────────────────────────
        display = bgr.copy()
        counts  = draw_detections(display, dets, conf_thresh, cls_filter, show_depth)

        t_now = time.time()
        fps_buf.append(1.0 / max(t_now - t_prev, 1e-6))
        t_prev = t_now
        if len(fps_buf) > 20:
            fps_buf.pop(0)
        fps = sum(fps_buf) / len(fps_buf)

        draw_hud(display, fps, conf_thresh, cls_filter, counts, model_path, paused)

        cv2.imshow('STI USV — Model Test (q=cikis)', display)

        # ── Tuş işleme ────────────────────────────────────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            fname = f'screenshot_{screenshot_n:04d}.jpg'
            cv2.imwrite(fname, display)
            print(f'[KAYDET] {fname}')
            screenshot_n += 1
        elif key == ord('a'):
            cls_filter = -1
            print('[FİLTRE] Tüm sınıflar')
        elif ord('0') <= key <= ord('4'):
            cls_filter = key - ord('0')
            print(f'[FİLTRE] Sadece: {CLS_NAMES.get(cls_filter, "?")}')
        elif key in (ord('+'), ord('=')):
            conf_thresh = min(0.95, conf_thresh + 0.05)
            print(f'[CONF] Eşik: {conf_thresh:.2f}')
        elif key == ord('-'):
            conf_thresh = max(0.05, conf_thresh - 0.05)
            print(f'[CONF] Eşik: {conf_thresh:.2f}')
        elif key == ord('d'):
            show_depth = not show_depth
            print(f'[DERİNLİK] {"Açık" if show_depth else "Kapalı"}')
        elif key == ord(' '):
            paused = not paused
            print(f'[{"DONDURULDU" if paused else "DEVAM"}]')

    cam.close()
    cv2.destroyAllWindows()
    print('Çıkıldı.')


if __name__ == '__main__':
    main()
