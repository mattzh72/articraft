from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _bell_shell():
    return _mesh(
        "bell_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.010, 0.000),
                (0.014, -0.006),
                (0.020, -0.016),
                (0.028, -0.030),
                (0.038, -0.050),
                (0.046, -0.073),
                (0.050, -0.094),
            ],
            [
                (0.000, -0.004),
                (0.006, -0.012),
                (0.013, -0.021),
                (0.023, -0.036),
                (0.032, -0.055),
                (0.040, -0.076),
                (0.043, -0.088),
            ],
            segments=56,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _beam_box_xz(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    depth: float,
):
    x0, y0, z0 = start
    x1, y1, z1 = end
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    angle_y = math.atan2(dx, dz)
    return Box((width, depth, length)), Origin(
        xyz=((x0 + x1) * 0.5, y0, (z0 + z1) * 0.5),
        rpy=(0.0, angle_y, 0.0),
    )


def _beam_box_yz(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    width: float,
    depth: float,
):
    x0, y0, z0 = start
    x1, y1, z1 = end
    dy = y1 - y0
    dz = z1 - z0
    length = math.hypot(dy, dz)
    angle_x = -math.atan2(dy, dz)
    return Box((width, depth, length)), Origin(
        xyz=(x0, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
        rpy=(angle_x, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_bell_tower", assets=ASSETS)

    warm_timber = model.material("warm_timber", rgba=(0.48, 0.33, 0.20, 1.0))
    dark_timber = model.material("dark_timber", rgba=(0.31, 0.21, 0.13, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.45, 0.20, 1.0))
    iron = model.material("iron", rgba=(0.22, 0.23, 0.25, 1.0))

    tower_frame = model.part("tower_frame")
    tower_frame.visual(
        Box((0.240, 0.320, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_timber,
        name="platform",
    )
    tower_frame.visual(
        Box((0.200, 0.270, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=warm_timber,
        name="deck",
    )

    post_centers = {
        "front_left": (-0.090, 0.130),
        "front_right": (0.090, 0.130),
        "rear_left": (-0.090, -0.130),
        "rear_right": (0.090, -0.130),
    }
    for name, (px, py) in post_centers.items():
        tower_frame.visual(
            Box((0.022, 0.022, 0.280)),
            origin=Origin(xyz=(px, py, 0.160)),
            material=warm_timber,
            name=f"{name}_post",
        )

    tower_frame.visual(
        Box((0.202, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.130, 0.060)),
        material=dark_timber,
        name="front_lower_beam",
    )
    tower_frame.visual(
        Box((0.202, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.130, 0.060)),
        material=dark_timber,
        name="rear_lower_beam",
    )
    tower_frame.visual(
        Box((0.018, 0.282, 0.018)),
        origin=Origin(xyz=(-0.090, 0.0, 0.060)),
        material=dark_timber,
        name="left_lower_beam",
    )
    tower_frame.visual(
        Box((0.018, 0.282, 0.018)),
        origin=Origin(xyz=(0.090, 0.0, 0.060)),
        material=dark_timber,
        name="right_lower_beam",
    )

    tower_frame.visual(
        Box((0.202, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.130, 0.286)),
        material=dark_timber,
        name="front_upper_beam",
    )
    tower_frame.visual(
        Box((0.202, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.130, 0.286)),
        material=dark_timber,
        name="rear_upper_beam",
    )
    tower_frame.visual(
        Box((0.018, 0.282, 0.018)),
        origin=Origin(xyz=(-0.090, 0.0, 0.286)),
        material=dark_timber,
        name="left_upper_beam",
    )
    tower_frame.visual(
        Box((0.018, 0.282, 0.018)),
        origin=Origin(xyz=(0.090, 0.0, 0.286)),
        material=dark_timber,
        name="right_upper_beam",
    )

    knee_braces_xz = {
        "front_left_brace": ((-0.090, 0.130, 0.165), (-0.035, 0.130, 0.245)),
        "front_right_brace": ((0.090, 0.130, 0.165), (0.035, 0.130, 0.245)),
        "rear_left_brace": ((-0.090, -0.130, 0.165), (-0.035, -0.130, 0.245)),
        "rear_right_brace": ((0.090, -0.130, 0.165), (0.035, -0.130, 0.245)),
    }
    for name, (start, end) in knee_braces_xz.items():
        geometry, origin = _beam_box_xz(start, end, width=0.016, depth=0.014)
        tower_frame.visual(geometry, origin=origin, material=warm_timber, name=name)

    knee_braces_yz = {
        "left_front_brace": ((-0.090, 0.130, 0.165), (-0.090, 0.055, 0.245)),
        "left_rear_brace": ((-0.090, -0.130, 0.165), (-0.090, -0.055, 0.245)),
        "right_front_brace": ((0.090, 0.130, 0.165), (0.090, 0.055, 0.245)),
        "right_rear_brace": ((0.090, -0.130, 0.165), (0.090, -0.055, 0.245)),
    }
    for name, (start, end) in knee_braces_yz.items():
        geometry, origin = _beam_box_yz(start, end, width=0.014, depth=0.016)
        tower_frame.visual(geometry, origin=origin, material=warm_timber, name=name)

    tower_frame.visual(
        Cylinder(radius=0.010, length=0.168),
        origin=Origin(xyz=(0.0, 0.0, 0.278), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_timber,
        name="crossbar",
    )

    roof_members = {
        "front_left_rafter": ((-0.082, 0.135, 0.289), (-0.018, 0.135, 0.385)),
        "front_right_rafter": ((0.082, 0.135, 0.289), (0.018, 0.135, 0.385)),
        "rear_left_rafter": ((-0.082, -0.135, 0.289), (-0.018, -0.135, 0.385)),
        "rear_right_rafter": ((0.082, -0.135, 0.289), (0.018, -0.135, 0.385)),
    }
    for name, (start, end) in roof_members.items():
        geometry, origin = _beam_box_xz(start, end, width=0.016, depth=0.016)
        tower_frame.visual(geometry, origin=origin, material=warm_timber, name=name)

    tower_frame.visual(
        Box((0.014, 0.270, 0.014)),
        origin=Origin(xyz=(-0.026, 0.0, 0.383)),
        material=dark_timber,
        name="left_peak_tie",
    )
    tower_frame.visual(
        Box((0.014, 0.270, 0.014)),
        origin=Origin(xyz=(0.026, 0.0, 0.383)),
        material=dark_timber,
        name="right_peak_tie",
    )
    tower_frame.inertial = Inertial.from_geometry(
        Box((0.240, 0.320, 0.420)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    bell = model.part("bell")
    bell.visual(
        Cylinder(radius=0.0125, length=0.056),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="pivot_sleeve",
    )
    bell.visual(
        Box((0.020, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=iron,
        name="hanger_strap",
    )
    bell.visual(
        _bell_shell(),
        origin=Origin(xyz=(0.0, 0.0, -0.044)),
        material=bronze,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.004, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        material=iron,
        name="clapper_rod",
    )
    bell.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=iron,
        name="clapper_bob",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.145),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, -0.072)),
    )

    model.articulation(
        "bell_swing",
        ArticulationType.REVOLUTE,
        parent=tower_frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-0.7,
            upper=0.7,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower_frame = object_model.get_part("tower_frame")
    bell = object_model.get_part("bell")
    bell_swing = object_model.get_articulation("bell_swing")

    platform = tower_frame.get_visual("platform")
    crossbar = tower_frame.get_visual("crossbar")
    front_left_post = tower_frame.get_visual("front_left_post")
    rear_left_post = tower_frame.get_visual("rear_left_post")
    left_upper_beam = tower_frame.get_visual("left_upper_beam")
    right_upper_beam = tower_frame.get_visual("right_upper_beam")
    front_left_rafter = tower_frame.get_visual("front_left_rafter")
    front_right_rafter = tower_frame.get_visual("front_right_rafter")
    rear_left_rafter = tower_frame.get_visual("rear_left_rafter")
    rear_right_rafter = tower_frame.get_visual("rear_right_rafter")

    pivot_sleeve = bell.get_visual("pivot_sleeve")
    bell_shell = bell.get_visual("bell_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.allow_overlap(bell, tower_frame, reason="bell hanger sleeve nests around the timber pivot rod")
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(bell, tower_frame, axes="xy", max_dist=0.001)
    ctx.expect_overlap(
        bell,
        tower_frame,
        axes="xz",
        elem_a=pivot_sleeve,
        elem_b=crossbar,
        min_overlap=0.020,
    )
    ctx.expect_gap(
        bell,
        tower_frame,
        axis="z",
        min_gap=0.080,
        positive_elem=bell_shell,
        negative_elem=platform,
        name="bell_hangs_above_platform",
    )
    ctx.expect_gap(
        bell,
        tower_frame,
        axis="x",
        min_gap=0.015,
        positive_elem=bell_shell,
        negative_elem=left_upper_beam,
        name="bell_clears_left_frame",
    )
    ctx.expect_gap(
        tower_frame,
        bell,
        axis="x",
        min_gap=0.015,
        positive_elem=right_upper_beam,
        negative_elem=bell_shell,
        name="bell_clears_right_frame",
    )
    ctx.expect_gap(
        tower_frame,
        bell,
        axis="y",
        min_gap=0.020,
        positive_elem=front_left_post,
        negative_elem=bell_shell,
        name="bell_clears_front_post_line",
    )
    ctx.expect_gap(
        bell,
        tower_frame,
        axis="y",
        min_gap=0.020,
        positive_elem=bell_shell,
        negative_elem=rear_left_post,
        name="bell_clears_rear_post_line",
    )
    ctx.expect_gap(
        tower_frame,
        tower_frame,
        axis="x",
        min_gap=0.010,
        positive_elem=front_right_rafter,
        negative_elem=front_left_rafter,
        name="front_peak_stays_open",
    )
    ctx.expect_gap(
        tower_frame,
        tower_frame,
        axis="x",
        min_gap=0.010,
        positive_elem=rear_right_rafter,
        negative_elem=rear_left_rafter,
        name="rear_peak_stays_open",
    )

    with ctx.pose({bell_swing: 0.45}):
        ctx.expect_overlap(
            bell,
            tower_frame,
            axes="xz",
            elem_a=pivot_sleeve,
            elem_b=crossbar,
            min_overlap=0.020,
        )
        ctx.expect_gap(
            tower_frame,
            bell,
            axis="y",
            min_gap=0.010,
            positive_elem=front_left_post,
            negative_elem=bell_shell,
            name="bell_clears_front_post_line_swung_forward",
        )
        ctx.expect_gap(
            bell,
            tower_frame,
            axis="z",
            min_gap=0.045,
            positive_elem=bell_shell,
            negative_elem=platform,
            name="bell_stays_above_platform_swung_forward",
        )

    with ctx.pose({bell_swing: -0.45}):
        ctx.expect_gap(
            bell,
            tower_frame,
            axis="y",
            min_gap=0.010,
            positive_elem=bell_shell,
            negative_elem=rear_left_post,
            name="bell_clears_rear_post_line_swung_back",
        )
        ctx.expect_gap(
            bell,
            tower_frame,
            axis="z",
            min_gap=0.045,
            positive_elem=bell_shell,
            negative_elem=platform,
            name="bell_stays_above_platform_swung_back",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
