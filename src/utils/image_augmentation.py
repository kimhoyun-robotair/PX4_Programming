#!/usr/bin/env python3
import cv2
import numpy as np
from pathlib import Path
from itertools import product
import random
import math

# ───── 사용자 설정 ─────
SRC_DIR = Path("/home/kimhoyun/for_dataset/image")   # 원본 폴더
DST_DIR = Path("aug_dataset")                        # 저장 폴더
DST_DIR.mkdir(parents=True, exist_ok=True)

BLUR_KERNELS    = [(3, 3), (5, 5)]                  # 2
GAUSS_NOISE_STD = [3, 7, 11]                        # 3
TRANSLATIONS    = [0, 5]                            # 2
ROT_ANGLES      = list(range(-20, 21, 10))          # 5  (-20,-10,0,10,20)
PERS_WARP_RATIO = [0.05, 0.10]                      # 2
MAX_OUTPUTS     = 10_000
LOG_STEP        = 1_000
# ───────────────────────

def random_perspective(img, max_ratio):
    h, w = img.shape[:2]
    d = max_ratio * min(h, w)
    src = np.float32([[0, 0], [w-1, 0], [0, h-1], [w-1, h-1]])
    dst = src + np.random.uniform(-d, d, (4, 2)).astype(np.float32)
    M   = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(img, M, (w, h))

# 1. 변환 조합 목록(총 120개) 생성
combo_list = list(product(
    range(len(BLUR_KERNELS)),
    range(len(GAUSS_NOISE_STD)),
    TRANSLATIONS,
    ROT_ANGLES,
    range(len(PERS_WARP_RATIO))
))

# 2. 원본 이미지 목록
img_paths = [p for p in SRC_DIR.iterdir()
             if p.suffix.lower() in {'.jpg', '.jpeg', '.png'}]
img_paths.sort()
print(f"▶ 원본 {len(img_paths)}장, 가능한 조합 {len(combo_list)}개")

# 3. 이미지 1장당 몇 개 조합을 만들지 계산
quota = math.ceil(MAX_OUTPUTS / len(img_paths))      # 예: 350장 → 29
quota = min(quota, len(combo_list))                  # 상한 120
print(f"▶ 이미지당 최대 {quota}개 증강본을 생성합니다.")

save_cnt = 0
for img_path in img_paths:
    if save_cnt >= MAX_OUTPUTS:
        break

    img = cv2.imread(str(img_path))
    if img is None:
        print(f"  └─ 불러오기 실패: {img_path}")
        continue

    rows, cols, ch = img.shape
    base = img_path.stem
    ext  = img_path.suffix.lower()

    # 4. 조합 중 quota개 무작위 선택
    sampled_combos = random.sample(combo_list, quota)

    for (blur_idx,
         noise_idx,
         t_val,
         angle,
         pers_idx) in sampled_combos:

        # -------- 블러 --------
        img_aug = cv2.blur(img, BLUR_KERNELS[blur_idx])

        # -------- 노이즈 --------
        std = GAUSS_NOISE_STD[noise_idx]
        noise = np.random.normal(0, std, (rows, cols, ch))
        img_aug = np.clip(img_aug.astype(np.float32) + noise,
                          0, 255).astype(np.uint8)

        # -------- 평행 이동 --------
        M_t = np.float32([[1, 0, t_val], [0, 1, t_val]])
        img_aug = cv2.warpAffine(img_aug, M_t, (cols, rows))

        # -------- 회전 --------
        M_r = cv2.getRotationMatrix2D((cols/2, rows/2), angle, 1)
        img_aug = cv2.warpAffine(img_aug, M_r, (cols, rows))

        # -------- 원근 --------
        img_aug = random_perspective(img_aug,
                                     PERS_WARP_RATIO[pers_idx])

        # -------- 저장 --------
        out_name = (f"{base}"
                    f"_b{BLUR_KERNELS[blur_idx][0]}"
                    f"_n{noise_idx}"
                    f"_t{t_val}"
                    f"_r{angle}"
                    f"_p{pers_idx}{ext}")
        cv2.imwrite(str(DST_DIR / out_name), img_aug)
        save_cnt += 1

        # 5. 진행 로그 & 상한 체크
        if save_cnt % LOG_STEP == 0:
            print(f"  ▶ {save_cnt:,}장 생성")

        if save_cnt >= MAX_OUTPUTS:
            break

print(f"✅ 완료! 최종 {save_cnt:,}장 저장했습니다.")
