from isaaclab.utils import configclass
from ..parkour_terrain_generator_cfg import ParkourSubTerrainBaseCfg
from . import extreme_parkour_terrians


@configclass
class ExtremeParkourRoughTerrainCfg(ParkourSubTerrainBaseCfg):
    apply_roughness: bool = True
    apply_flat: bool = False
    downsampled_scale: float | None = 0.075
    noise_range: tuple[float, float] = (0.02, 0.06)
    noise_step: float = 0.005
    x_range: tuple[float, float] = (0.8, 1.5)
    y_range: tuple[float, float] = (-0.4, 0.4)
    half_valid_width: tuple[float, float] = (0.6, 1.2)
    pad_width: float = 0.1
    pad_height: float = 0.0


@configclass
class ExtremeParkourGapTerrainCfg(ExtremeParkourRoughTerrainCfg):
    function = extreme_parkour_terrians.parkour_gap_terrain
    horizontal_scale: float = 0.02
    slope_threshold: float | None = 3.75
    downsampled_scale: float | None = None
    gap_size: str = "0.1 + 0.7*difficulty"
    gap_width: str | None = None
    gap_depth: tuple[float, float] = (2.2, 2.6)
    height_drop_per_gap: str | float = "0.05 + 0.05 * difficulty"


@configclass
class ExtremeParkourHurdleTerrainCfg(ExtremeParkourRoughTerrainCfg):
    function = extreme_parkour_terrians.parkour_hurdle_terrain
    stone_len: str = "0.1 + 0.3 * difficulty"
    hurdle_height_range: str = "0.1 + 0.1 * difficulty, 0.15 + 0.15 * difficulty"
    pole_width: float = 0.2


@configclass
class ExtremeParkourStepTerrainCfg(ExtremeParkourRoughTerrainCfg):
    function = extreme_parkour_terrians.parkour_step_terrain
    step_height: str = "0.1 + 0.35*difficulty"


@configclass
class ExtremeParkourTerrainCfg(ExtremeParkourRoughTerrainCfg):
    function = extreme_parkour_terrians.parkour_terrain
    pit_depth: tuple[float, float] = (0.2, 1)
    stone_width: float = 1.0
    last_stone_len: float = 1.6
    x_range: str = "-0.1, 0.1+0.3*difficulty"
    y_range: str = "0.2, 0.3+0.1*difficulty"
    stone_len: str = "0.9 - 0.3*difficulty, 1 - 0.2*difficulty"
    incline_height: str = "0.25*difficulty"
    last_incline_height: str = "incline_height + 0.1 - 0.1*difficulty"


@configclass
class ExtremeParkourDemoTerrainCfg(ExtremeParkourRoughTerrainCfg):
    function = extreme_parkour_terrians.parkour_demo_terrain


@configclass
class ExtremeParkourWallTerrainCfg(ExtremeParkourRoughTerrainCfg):
    """Configuration for wall jumping terrain - robot needs to jump over a wall."""

    function = extreme_parkour_terrians.parkour_wall_terrain
    wall_thickness: str = "0.05 + 0.05 * difficulty"  # 墙的厚度 (米)
    wall_height_range: str = (
        "0.2 + 0.1 * difficulty, 0.3 + 0.2 * difficulty"  # 墙的高度范围
    )
    x_range: tuple[float, float] = (1.5, 3.0)  # 墙之间的距离
    allow_bypass: bool = False  # 是否允许绕过墙（True=可以绕过, False=不可绕过）
    add_side_pits: bool = False  # 墙两侧是否添加深坑（防止绕行）
    pit_depth: tuple[float, float] = (0.3, 0.5)  # 深坑深度范围 (米)


@configclass
class ExtremeParkourHurdleTriMeshTerrainCfg(ExtremeParkourRoughTerrainCfg):
    """Configuration for hurdle terrain with trimesh generation - robot needs to crawl under the bar."""

    function = extreme_parkour_terrians.parkour_hurdle_terrain_trimesh

    # 正方形横截面尺寸
    pole_size: str = "0.08 + 0.04 * difficulty"  # 柱子边长 (米)
    bar_size: str = "0.06 + 0.03 * difficulty"  # 横杆边长 (米)

    # 跨栏整体高度 (从地面到横杆底部的距离)
    hurdle_height_range: str = "0.25 + 0.15 * difficulty, 0.35 + 0.25 * difficulty"

    # 跨栏间距和位置
    x_range: tuple[float, float] = (1.5, 2.5)  # 跨栏之间的纵向距离 (米)
    y_range: tuple[float, float] = (-0.3, 0.3)  # 跨栏的横向偏移范围 (米)

    # 通道参数
    half_valid_width: tuple[float, float] = (0.4, 0.6)  # 两个柱子之间通道的半宽度 (米)


@configclass
class ExtremeParkourSlopeTerrainCfg(ExtremeParkourRoughTerrainCfg):
    """
    侧向进入斜坡地形：
    - 地面整体是左右倾斜的"斜坡带"
    - 机器人沿 x 方向前进，沿着等高线行走，不直接爬坡
    - 多个斜坡段在 x 方向串联，每段斜率与长度都从范围中采样
    """

    function = extreme_parkour_terrians.parkour_slope_terrain

    # 斜率范围（单位：m 高度 / m 横向），允许正负，难度越高斜率范围略扩大
    # 例如 difficulty=1 时，大约在 [-0.2, 0.2] 左右
    slope_range: str = "-0.15 - 0.05 * difficulty, 0.15 + 0.05 * difficulty"

    # 每一段斜坡在机器人行走方向上的长度范围（单位：m）
    # 难度越高，平均段长略变大
    segment_width_range: str = "0.8 + 0.2 * difficulty, 1.6 + 0.4 * difficulty"
    noise_range: tuple[float, float] = (0.01, 0.1)


@configclass
class ExtremeParkourFixedGapTerrainCfg(ExtremeParkourRoughTerrainCfg):
    """
    间隙地形（随难度动态调整）：
    - 平台长度：700mm -> 300mm (随难度递减)
    - 间隙长度：200mm -> 100mm (随难度递减)
    - 通道宽度：1000mm = 1.0m (固定)
    - 平台-间隙-平台-间隙循环往复

    注意：使用 internal_horizontal_scale (0.05) 在内部生成高精度地形，
    然后自动重采样到主网格的 horizontal_scale，以确保精确尺寸且不会 OOM
    """

    function = extreme_parkour_terrians.parkour_fixed_gap_terrain
    internal_horizontal_scale: float = 0.05  # 内部高精度网格 (5cm)，仅用于此地形
    platform_length: str | float = (
        0.7  # 平台长度 (米)，可使用表达式如 '0.7 - 0.4 * difficulty'
    )
    gap_length: str | float = (
        0.2  # 间隙长度 (米)，可使用表达式如 '0.2 - 0.1 * difficulty'
    )
    walkway_width: float = 1.0  # 通道宽度 (米) = 1000mm
    gap_depth: tuple[float, float] = (0.4, 1.6)  # 间隙深度 (米)
    apply_roughness: bool = False  # 不应用粗糙表面，保持平整
