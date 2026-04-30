#!/usr/bin/env python3
"""
model_benchmark.py — STI USV Model Değerlendirme Aracı
=======================================================
İki mod:
  1. live   → ZED kamerasından 300 kare yakalar, istatistik üretir
  2. folder → Bir klasördeki .jpg/.png dosyaları üzerinde test eder

Kullanım:
  python3 model_benchmark.py live
  python3 model_benchmark.py folder /path/to/images
  python3 model_benchmark.py live --model son.pt --conf 0.35 --frames 200

Çıktı:
  - Terminal raporu (per-class istatistikler)
  - benchmark_results/  klasörüne annotated görüntüler (yanlış tespit + düşük conf)
  - benchmark_report.txt özet dosyası
"""

import argparse
import os
import sys
import time
import json
from collections import defaultdict

import cv2
import numpy as np

# ── Sınıf tanımları ────────────────────────────────────────────────────────────
CLS_NAMES  = {0: 'Black', 1: 'Green', 2: 'Orange', 3: 'Red', 4: 'Yellow'}
CLS_COLORS = {
    0: (50,  50,  50),
    1: (0,   200, 50),
    2: (0,   140, 255),
    3: (0,   0,   220),
    4: (0,   220, 220),
}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR  = os.path.join(os.path.dirname(SCRIPT_DIR), 'models')
OUT_DIR    = 'benchmark_results'


def load_model(model_arg):
    from ultralytics import YOLO
    candidates = [
        model_arg,
        os.path.join(MODEL_DIR, 'son.engine'),
        os.path.join(MODEL_DIR, 'son.pt'),
    ]
    for path in candidates:
        if path and os.path.exists(path):
            print(f'[MODEL] Yükleniyor: {path}')
            m = YOLO(path, task='detect')
            dummy = np.zeros((180, 320, 3), dtype=np.uint8)
            m.predict(dummy, device='0', verbose=False, conf=0.1)
            print(f'[MODEL] ✓ sınıflar={m.names}')
            return m, path
    raise RuntimeError('Model bulunamadı!')


def run_inference(model, bgr, conf_thresh):
    results = model.predict(bgr, device='0', verbose=False, conf=0.05)
    dets = []
    if results and results[0].boxes is not None:
        for box in results[0].boxes:
            x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].cpu().numpy()]
            cls_id = int(box.cls[0].item())
            conf   = float(box.conf[0].item())
            w_box  = x2 - x1
            h_box  = y2 - y1
            dets.append({
                'cls': cls_id, 'conf': conf,
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'cx': (x1+x2)//2, 'cy': (y1+y2)//2,
                'area': w_box * h_box,
                'aspect': w_box / max(h_box, 1),
            })
    return dets


def annotate(frame, dets, conf_thresh):
    out = frame.copy()
    for d in dets:
        col  = CLS_COLORS.get(d['cls'], (200,200,200))
        name = CLS_NAMES.get(d['cls'], str(d['cls']))
        cv2.rectangle(out, (d['x1'],d['y1']), (d['x2'],d['y2']), col, 2)
        label = f"{name} {d['conf']:.2f}"
        cv2.putText(out, label, (d['x1'], max(d['y1']-5,12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)
    return out


def print_report(stats, conf_thresh, model_path, mode, n_frames, elapsed):
    print()
    print('╔══════════════════════════════════════════════════════════════╗')
    print('║              STI USV — BENCHMARK RAPORU                     ║')
    print('╠══════════════════════════════════════════════════════════════╣')
    print(f'  Model  : {os.path.basename(model_path)}')
    print(f'  Mod    : {mode}')
    print(f'  Kare   : {n_frames}')
    print(f'  Süre   : {elapsed:.1f}s  ({n_frames/elapsed:.1f} FPS)')
    print(f'  Conf   : ≥{conf_thresh}')
    print()

    total_dets = sum(stats['cls_count'].values())
    print(f'  Toplam tespit       : {total_dets}  ({total_dets/n_frames:.2f} / kare)')
    print(f'  Tespit olan kare    : {stats["frames_with_det"]}  '
          f'({100*stats["frames_with_det"]/n_frames:.1f}%)')
    print(f'  Boş kare (0 tespit) : {stats["frames_empty"]}  '
          f'({100*stats["frames_empty"]/n_frames:.1f}%)')
    print()

    print('  ── Sınıf Başına İstatistik ──')
    print(f'  {"Sınıf":<10} {"Tespit":>8} {"Ort.Conf":>10} {"Min.Conf":>10} {"Ort.Alan(px²)":>14}')
    print('  ' + '─'*56)
    for cls_id in sorted(CLS_NAMES.keys()):
        name  = CLS_NAMES[cls_id]
        count = stats['cls_count'].get(cls_id, 0)
        if count == 0:
            print(f'  {name:<10} {"0":>8}  {"—":>9}  {"—":>9}  {"—":>13}')
            continue
        confs = stats['cls_confs'].get(cls_id, [])
        areas = stats['cls_areas'].get(cls_id, [])
        print(f'  {name:<10} {count:>8}  '
              f'{sum(confs)/len(confs):>9.3f}  '
              f'{min(confs):>9.3f}  '
              f'{sum(areas)/len(areas):>13.0f}')

    print()
    print('  ── Confidence Dağılımı (tüm sınıflar) ──')
    all_confs = []
    for c in stats['cls_confs'].values():
        all_confs.extend(c)
    if all_confs:
        buckets = [(0.0,0.3),(0.3,0.4),(0.4,0.5),(0.5,0.6),(0.6,0.7),(0.7,0.8),(0.8,0.9),(0.9,1.0)]
        for lo, hi in buckets:
            cnt = sum(1 for c in all_confs if lo <= c < hi)
            bar = '█' * (cnt * 30 // max(len(all_confs),1))
            print(f'  {lo:.1f}-{hi:.1f}: {bar:<30} {cnt:>4}')

    print()
    # Uyarılar
    warnings = []
    for cls_id, confs in stats['cls_confs'].items():
        if confs:
            avg = sum(confs)/len(confs)
            if avg < 0.55:
                warnings.append(f'⚠  {CLS_NAMES[cls_id]}: ortalama conf düşük ({avg:.2f}) — '
                                 f'bu sınıfı model zor tanıyor')
    if stats['frames_empty'] / n_frames > 0.3:
        warnings.append(f'⚠  Karelerin %{100*stats["frames_empty"]/n_frames:.0f}\'inde hiç tespit yok — '
                         f'uzak mesafe veya yanlış conf eşiği?')
    multi = [f for f in stats['multi_det_frames'] if f > 1]
    if len(multi) > n_frames * 0.1:
        warnings.append(f'⚠  Karelerin %{100*len(multi)/n_frames:.0f}\'inde 2+ tespit — '
                         f'false positive veya NMS sorunu olabilir')

    if warnings:
        print('  ── Uyarılar ──')
        for w in warnings:
            print(f'  {w}')
    else:
        print('  ✓ Belirgin sorun tespit edilmedi.')

    print('╚══════════════════════════════════════════════════════════════╝')


def benchmark_live(model, model_path, conf_thresh, n_frames):
    import pyzed.sl as sl

    cam    = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.HD720
    params.camera_fps        = 30
    params.depth_mode        = sl.DEPTH_MODE.NONE
    status = cam.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f'ZED açılamadı: {status}')

    img_zed = sl.Mat()
    os.makedirs(OUT_DIR, exist_ok=True)

    stats = defaultdict(lambda: defaultdict(list))
    stats = {
        'cls_count': defaultdict(int),
        'cls_confs':  defaultdict(list),
        'cls_areas':  defaultdict(list),
        'frames_with_det': 0,
        'frames_empty': 0,
        'multi_det_frames': [],
    }

    print(f'\n[BENCHMARK] {n_frames} kare yakalanacak — lütfen bekle...')
    print('[BENCHMARK] Kamerayı şamandıralara doğrult!')
    print('[BENCHMARK] q tuşuna basarak iptal edebilirsin.\n')

    cv2.namedWindow('Benchmark', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Benchmark', 960, 540)

    saved = 0
    t_start = time.time()

    for i in range(n_frames):
        if cam.grab() != sl.ERROR_CODE.SUCCESS:
            continue
        cam.retrieve_image(img_zed, sl.VIEW.LEFT)
        bgr = cv2.cvtColor(img_zed.get_data(), cv2.COLOR_BGRA2BGR)

        dets = run_inference(model, bgr, conf_thresh)
        dets_thresh = [d for d in dets if d['conf'] >= conf_thresh]

        stats['multi_det_frames'].append(len(dets_thresh))
        if dets_thresh:
            stats['frames_with_det'] += 1
            for d in dets_thresh:
                stats['cls_count'][d['cls']] += 1
                stats['cls_confs'][d['cls']].append(d['conf'])
                stats['cls_areas'][d['cls']].append(d['area'])
        else:
            stats['frames_empty'] += 1

        # Düşük confidence veya çok tespit varsa kaydet (ilk 20)
        low_conf = any(d['conf'] < 0.45 for d in dets_thresh)
        many_det = len(dets_thresh) >= 3
        if (low_conf or many_det) and saved < 20:
            tag = 'lowconf' if low_conf else 'multidet'
            ann = annotate(bgr, dets_thresh, conf_thresh)
            fname = os.path.join(OUT_DIR, f'{tag}_{saved:03d}.jpg')
            cv2.imwrite(fname, ann)
            saved += 1

        # Canlı önizleme
        ann = annotate(bgr, dets_thresh, conf_thresh)
        prog = f'Kare {i+1}/{n_frames}'
        cv2.putText(ann, prog, (8, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
        cv2.imshow('Benchmark', ann)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            n_frames = i + 1
            break

    elapsed = time.time() - t_start
    cam.close()
    cv2.destroyAllWindows()
    print_report(stats, conf_thresh, model_path, 'live', n_frames, elapsed)


def benchmark_folder(model, model_path, conf_thresh, folder):
    exts = {'.jpg', '.jpeg', '.png', '.bmp'}
    files = [os.path.join(folder, f) for f in sorted(os.listdir(folder))
             if os.path.splitext(f)[1].lower() in exts]
    if not files:
        print(f'[HATA] {folder} içinde görüntü yok!')
        return

    os.makedirs(OUT_DIR, exist_ok=True)
    stats = {
        'cls_count': defaultdict(int),
        'cls_confs':  defaultdict(list),
        'cls_areas':  defaultdict(list),
        'frames_with_det': 0,
        'frames_empty': 0,
        'multi_det_frames': [],
    }

    print(f'[BENCHMARK] {len(files)} dosya işleniyor...')
    t_start = time.time()

    for i, fpath in enumerate(files):
        bgr = cv2.imread(fpath)
        if bgr is None:
            continue
        dets = run_inference(model, bgr, conf_thresh)
        dets_thresh = [d for d in dets if d['conf'] >= conf_thresh]

        stats['multi_det_frames'].append(len(dets_thresh))
        if dets_thresh:
            stats['frames_with_det'] += 1
            for d in dets_thresh:
                stats['cls_count'][d['cls']] += 1
                stats['cls_confs'][d['cls']].append(d['conf'])
                stats['cls_areas'][d['cls']].append(d['area'])
        else:
            stats['frames_empty'] += 1

        ann = annotate(bgr, dets_thresh, conf_thresh)
        cv2.imwrite(os.path.join(OUT_DIR, f'result_{i:04d}_{os.path.basename(fpath)}'), ann)

        if (i+1) % 10 == 0:
            print(f'  {i+1}/{len(files)}...')

    elapsed = time.time() - t_start
    print_report(stats, conf_thresh, model_path, f'folder:{folder}', len(files), elapsed)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['live','folder'], help='live veya folder')
    parser.add_argument('path', nargs='?', default=None, help='folder modu için dizin yolu')
    parser.add_argument('--model',  default=None, help='Model yolu')
    parser.add_argument('--conf',   type=float, default=0.35)
    parser.add_argument('--frames', type=int,   default=300, help='live mod kare sayısı')
    args = parser.parse_args()

    model, model_path = load_model(args.model)

    if args.mode == 'live':
        benchmark_live(model, model_path, args.conf, args.frames)
    else:
        if not args.path:
            print('[HATA] folder modu için dizin yolu gerekli')
            sys.exit(1)
        benchmark_folder(model, model_path, args.conf, args.path)


if __name__ == '__main__':
    main()
