import cv2
import re
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter


def load_pfm(path):
    with open(path, 'rb') as f:
        # 1) ヘッダ
        header = f.readline().decode('ascii').rstrip()
        color = header == 'PF'          # RGB = 'PF', グレースケール = 'Pf'
        if header not in ('PF', 'Pf'):
            raise ValueError(f'Not a PFM file: {path}')

        # 2) コメント行を飛ばしつつサイズを読む
        dims_line = f.readline().decode('ascii')
        while dims_line.startswith('#'):
            dims_line = f.readline().decode('ascii')
        w, h = map(int, re.findall(r'\d+', dims_line))

        # 3) スケール（負ならリトルエンディアン）
        scale = float(f.readline().decode('ascii').rstrip())
        endian = '<' if scale < 0 else '>'
        data = np.fromfile(f, endian + 'f4')   # float32

    # 4) 配列整形
    shape = (h, w, 3) if color else (h, w)
    data = np.reshape(data, shape)             # 1 本で読み込んで reshape
    data = np.flipud(data)                     # 上下反転で通常の向きへ
    return data


# img_A = load_pfm('../build/results/cos.pfm')
# img_B = load_pfm('../build/results/nee.pfm')

img_A = cv2.imread('../build/results/cos.pfm', cv2.IMREAD_UNCHANGED).astype(np.float32)
img_B = cv2.imread('../build/results/nee.pfm', cv2.IMREAD_UNCHANGED).astype(np.float32)
# img_A = cv2.imread('../build/compare/length_2/cos.png', cv2.IMREAD_UNCHANGED).astype(np.float32)
# img_B = cv2.imread('../build/compare/length_2/bdpt.png', cv2.IMREAD_UNCHANGED).astype(np.float32)

print(f"img_A.shape: {img_A.shape}")
print(f"img_B.shape: {img_B.shape}")

sigma_px = 3

diff = img_A - img_B
diff_gaussian = gaussian_filter(diff, sigma=(sigma_px, sigma_px, 0))
abs_diff_gaussian = np.abs(diff_gaussian)

plt.imshow(diff, vmin=0, vmax=1)
plt.axis('off')
plt.tight_layout()
plt.savefig('diff.png', dpi=200)
plt.show()
plt.close()

plt.imshow(diff_gaussian, vmin=0, vmax=1)
plt.axis('off')
plt.tight_layout()
plt.savefig('diff_gaussian.png', dpi=200)
plt.show()
plt.close()

plt.imshow(abs_diff_gaussian, vmin=0, vmax=1)
plt.axis('off')
plt.tight_layout()
plt.savefig('abs_diff_gaussian.png', dpi=200)
plt.show()
plt.close()

# diff_r = img_A[..., 2] - img_B[..., 2]
# diff_g = img_A[..., 1] - img_B[..., 1]
# diff_b = img_A[..., 0] - img_B[..., 0]

# diff_r_gauss = gaussian_filter(diff_r, sigma=sigma_px)
# diff_g_gauss = gaussian_filter(diff_g, sigma=sigma_px)
# diff_b_gauss = gaussian_filter(diff_b, sigma=sigma_px)

# abs_diff_r_gauss = np.abs(diff_r_gauss)
# abs_diff_g_gauss = np.abs(diff_g_gauss)
# abs_diff_b_gauss = np.abs(diff_b_gauss)

# plt.gray()
# plt.imshow(diff_r,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('diff_r.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_g, vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('diff_g.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_b,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('diff_b.png', dpi=200)
# plt.show()
# plt.close()

# plt.gray()
# plt.imshow(diff_r_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('diff_r_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_g_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('diff_g_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(diff_b_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('diff_b_gauss.png', dpi=200)
# plt.show()
# plt.close()

# plt.gray()
# plt.imshow(abs_diff_r_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('abs_diff_r_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(abs_diff_g_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('abs_diff_g_gauss.png', dpi=200)
# plt.show()
# plt.close()
# plt.gray()
# plt.imshow(abs_diff_b_gauss,  vmin=0, vmax=1)
# plt.axis('off')
# plt.tight_layout()
# plt.savefig('abs_diff_b_gauss.png', dpi=200)
# plt.show()
# plt.close()