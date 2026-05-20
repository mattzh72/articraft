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
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _write_lathe_shell_mesh(
    name: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    segments: int = 56,
):
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(name))


def _write_cable_housing_mesh():
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    geom = tube_from_spline_points(
        [
            (0.0225, 0.0055, 0.041),
            (0.026, 0.006, 0.020),
            (0.033, 0.006, -0.008),
            (0.040, 0.006, -0.034),
            (0.046, 0.006, -0.058),
        ],
        radius=0.0030,
        samples_per_segment=18,
        radial_segments=18,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("dropper_cable_housing.obj"))


def _write_exposed_cable_mesh():
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)
    geom = tube_from_spline_points(
        [
            (0.002, 0.0, -0.030),
            (0.000, -0.001, -0.0295),
            (-0.0025, -0.0025, -0.0288),
            (-0.0045, -0.0040, -0.0278),
        ],
        radius=0.0009,
        samples_per_segment=12,
        radial_segments=12,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path("dropper_exposed_cable.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="external_cable_dropper_seatpost", assets=ASSETS)

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.17, 0.18, 1.0))
    hardware = model.material("hardware", rgba=(0.62, 0.64, 0.67, 1.0))
    bushing_bronze = model.material("bushing_bronze", rgba=(0.62, 0.50, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    cable_black = model.material("cable_black", rgba=(0.05, 0.05, 0.05, 1.0))

    outer_sleeve = model.part("outer_sleeve")
    outer_sleeve.visual(
        _write_lathe_shell_mesh(
            "dropper_outer_shell.obj",
            outer_profile=[
                (0.0162, 0.0),
                (0.0162, 0.170),
                (0.0172, 0.188),
                (0.0180, 0.220),
            ],
            inner_profile=[
                (0.0140, 0.0),
                (0.0140, 0.170),
                (0.0144, 0.188),
                (0.0148, 0.220),
            ],
            segments=72,
        ),
        material=anodized_black,
        name="outer_body_shell",
    )
    outer_sleeve.visual(
        _write_lathe_shell_mesh(
            "dropper_wiper_cap.obj",
            outer_profile=[
                (0.0164, 0.0),
                (0.0164, 0.012),
            ],
            inner_profile=[
                (0.0134, 0.0),
                (0.0134, 0.012),
            ],
            segments=64,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=satin_black,
        name="wiper_cap",
    )
    outer_sleeve.visual(
        Box((0.012, 0.0022, 0.018)),
        origin=Origin(xyz=(0.0, 0.0136, 0.210)),
        material=bushing_bronze,
        name="bushing_front",
    )
    outer_sleeve.visual(
        Box((0.012, 0.0022, 0.018)),
        origin=Origin(xyz=(0.0, -0.0136, 0.210)),
        material=bushing_bronze,
        name="bushing_rear",
    )
    outer_sleeve.visual(
        Box((0.0022, 0.012, 0.018)),
        origin=Origin(xyz=(0.0136, 0.0, 0.210)),
        material=bushing_bronze,
        name="bushing_right",
    )
    outer_sleeve.visual(
        Box((0.0022, 0.012, 0.018)),
        origin=Origin(xyz=(-0.0136, 0.0, 0.210)),
        material=bushing_bronze,
        name="bushing_left",
    )
    outer_sleeve.visual(
        _write_lathe_shell_mesh(
            "dropper_boot_lower.obj",
            outer_profile=[
                (0.0188, 0.0),
                (0.0181, 0.008),
                (0.0174, 0.016),
            ],
            inner_profile=[
                (0.0163, 0.0),
                (0.0161, 0.008),
                (0.0159, 0.016),
            ],
            segments=64,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.232)),
        material=rubber,
        name="boot_lower",
    )
    outer_sleeve.visual(
        _write_lathe_shell_mesh(
            "dropper_boot_mid.obj",
            outer_profile=[
                (0.0176, 0.0),
                (0.0170, 0.008),
                (0.0165, 0.015),
            ],
            inner_profile=[
                (0.0160, 0.0),
                (0.0158, 0.008),
                (0.0156, 0.015),
            ],
            segments=64,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.248)),
        material=rubber,
        name="boot_mid",
    )
    outer_sleeve.visual(
        _write_lathe_shell_mesh(
            "dropper_boot_upper.obj",
            outer_profile=[
                (0.0168, 0.0),
                (0.0164, 0.010),
                (0.0160, 0.018),
            ],
            inner_profile=[
                (0.0159, 0.0),
                (0.0157, 0.010),
                (0.0155, 0.018),
            ],
            segments=64,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.263)),
        material=rubber,
        name="boot",
    )
    outer_sleeve.visual(
        Box((0.010, 0.012, 0.020)),
        origin=Origin(xyz=(0.0208, 0.0, 0.055)),
        material=satin_black,
        name="lever_bracket_body",
    )
    outer_sleeve.visual(
        Box((0.008, 0.002, 0.018)),
        origin=Origin(xyz=(0.0295, -0.0070, 0.060)),
        material=satin_black,
        name="lever_bracket_left",
    )
    outer_sleeve.visual(
        Box((0.008, 0.002, 0.018)),
        origin=Origin(xyz=(0.0295, 0.0070, 0.060)),
        material=satin_black,
        name="lever_bracket_right",
    )
    outer_sleeve.visual(
        Box((0.007, 0.008, 0.012)),
        origin=Origin(xyz=(0.0196, 0.0055, 0.041)),
        material=satin_black,
        name="cable_stop",
    )
    outer_sleeve.visual(
        _write_cable_housing_mesh(),
        material=cable_black,
        name="cable_housing",
    )
    outer_sleeve.inertial = Inertial.from_geometry(
        Box((0.048, 0.040, 0.240)),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0127, length=0.400),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=anodized_black,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0152, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=satin_black,
        name="guide_collar",
    )
    inner_post.visual(
        Box((0.052, 0.052, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.302)),
        material=satin_black,
        name="lower_cradle",
    )
    inner_post.visual(
        Cylinder(radius=0.004, length=0.076),
        origin=Origin(xyz=(-0.020, 0.0, 0.307), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_rail",
    )
    inner_post.visual(
        Cylinder(radius=0.004, length=0.076),
        origin=Origin(xyz=(0.020, 0.0, 0.307), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_rail",
    )
    inner_post.visual(
        Box((0.014, 0.044, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, 0.311)),
        material=satin_black,
        name="left_upper_clamp",
    )
    inner_post.visual(
        Box((0.014, 0.044, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, 0.311)),
        material=satin_black,
        name="right_upper_clamp",
    )
    inner_post.visual(
        Cylinder(radius=0.0035, length=0.068),
        origin=Origin(xyz=(0.0, 0.018, 0.314), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="front_bolt",
    )
    inner_post.visual(
        Cylinder(radius=0.0035, length=0.068),
        origin=Origin(xyz=(0.0, -0.018, 0.314), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="rear_bolt",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.080, 0.076, 0.390)),
        mass=0.52,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )

    lever = model.part("cable_lever")
    lever.visual(
        Cylinder(radius=0.0024, length=0.0120),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="pivot_barrel",
    )
    lever.visual(
        Box((0.007, 0.005, 0.032)),
        origin=Origin(xyz=(0.002, 0.0, -0.015)),
        material=satin_black,
        name="lever_arm",
    )
    lever.visual(
        Cylinder(radius=0.0025, length=0.007),
        origin=Origin(xyz=(0.002, 0.0, -0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="cable_nipple",
    )
    lever.visual(
        _write_exposed_cable_mesh(),
        material=hardware,
        name="exposed_cable",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.020, 0.012, 0.036)),
        mass=0.05,
        origin=Origin(xyz=(0.008, 0.0, -0.014)),
    )

    model.articulation(
        "sleeve_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer_sleeve,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.20, lower=0.0, upper=0.070),
    )
    model.articulation(
        "sleeve_to_lever",
        ArticulationType.REVOLUTE,
        parent=outer_sleeve,
        child=lever,
        origin=Origin(xyz=(0.0305, 0.0, 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.60, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_sleeve = object_model.get_part("outer_sleeve")
    inner_post = object_model.get_part("inner_post")
    cable_lever = object_model.get_part("cable_lever")
    inner_slide = object_model.get_articulation("sleeve_to_inner_post")
    lever_joint = object_model.get_articulation("sleeve_to_lever")

    outer_body_shell = outer_sleeve.get_visual("outer_body_shell")
    wiper_cap = outer_sleeve.get_visual("wiper_cap")
    bushing_front = outer_sleeve.get_visual("bushing_front")
    bushing_right = outer_sleeve.get_visual("bushing_right")
    boot = outer_sleeve.get_visual("boot")
    boot_lower = outer_sleeve.get_visual("boot_lower")
    bracket_body = outer_sleeve.get_visual("lever_bracket_body")
    bracket_left = outer_sleeve.get_visual("lever_bracket_left")
    bracket_right = outer_sleeve.get_visual("lever_bracket_right")
    cable_stop = outer_sleeve.get_visual("cable_stop")
    cable_housing = outer_sleeve.get_visual("cable_housing")

    stanchion = inner_post.get_visual("stanchion")
    guide_collar = inner_post.get_visual("guide_collar")
    lower_cradle = inner_post.get_visual("lower_cradle")
    left_rail = inner_post.get_visual("left_rail")
    right_rail = inner_post.get_visual("right_rail")
    left_upper_clamp = inner_post.get_visual("left_upper_clamp")
    right_upper_clamp = inner_post.get_visual("right_upper_clamp")
    front_bolt = inner_post.get_visual("front_bolt")
    rear_bolt = inner_post.get_visual("rear_bolt")

    pivot_barrel = cable_lever.get_visual("pivot_barrel")
    lever_arm = cable_lever.get_visual("lever_arm")
    cable_nipple = cable_lever.get_visual("cable_nipple")
    exposed_cable = cable_lever.get_visual("exposed_cable")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128, overlap_tol=0.001)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(inner_post, outer_sleeve, axes="xy", max_dist=0.0015)
    ctx.expect_gap(
        inner_post,
        outer_sleeve,
        axis="z",
        positive_elem=guide_collar,
        negative_elem=wiper_cap,
        max_gap=0.0015,
        max_penetration=0.0002,
    )
    ctx.expect_gap(
        outer_sleeve,
        inner_post,
        axis="y",
        positive_elem=bushing_front,
        negative_elem=stanchion,
        max_gap=0.0004,
        max_penetration=0.0003,
    )
    ctx.expect_gap(
        outer_sleeve,
        inner_post,
        axis="x",
        positive_elem=bushing_right,
        negative_elem=stanchion,
        max_gap=0.0004,
        max_penetration=0.0003,
    )
    ctx.expect_overlap(
        inner_post,
        outer_sleeve,
        axes="xy",
        elem_a=guide_collar,
        elem_b=boot,
        min_overlap=0.004,
    )
    ctx.expect_gap(
        outer_sleeve,
        outer_sleeve,
        axis="z",
        positive_elem=boot_lower,
        negative_elem=wiper_cap,
        max_gap=0.0005,
        max_penetration=0.0,
    )
    ctx.expect_contact(
        outer_sleeve,
        outer_sleeve,
        elem_a=bracket_body,
        elem_b=outer_body_shell,
    )
    ctx.expect_contact(
        outer_sleeve,
        outer_sleeve,
        elem_a=cable_stop,
        elem_b=outer_body_shell,
    )
    ctx.expect_contact(
        outer_sleeve,
        outer_sleeve,
        elem_a=cable_housing,
        elem_b=cable_stop,
    )
    ctx.expect_overlap(
        inner_post,
        inner_post,
        axes="xy",
        elem_a=left_upper_clamp,
        elem_b=left_rail,
        min_overlap=0.006,
    )
    ctx.expect_overlap(
        inner_post,
        inner_post,
        axes="xy",
        elem_a=right_upper_clamp,
        elem_b=right_rail,
        min_overlap=0.006,
    )
    ctx.expect_overlap(
        inner_post,
        inner_post,
        axes="xy",
        elem_a=lower_cradle,
        elem_b=left_rail,
        min_overlap=0.006,
    )
    ctx.expect_overlap(
        inner_post,
        inner_post,
        axes="xy",
        elem_a=lower_cradle,
        elem_b=right_rail,
        min_overlap=0.006,
    )
    ctx.expect_gap(
        inner_post,
        inner_post,
        axis="x",
        positive_elem=right_rail,
        negative_elem=left_rail,
        min_gap=0.028,
    )
    ctx.expect_gap(
        inner_post,
        inner_post,
        axis="y",
        positive_elem=front_bolt,
        negative_elem=rear_bolt,
        min_gap=0.024,
    )
    ctx.expect_overlap(
        inner_post,
        inner_post,
        axes="xz",
        elem_a=front_bolt,
        elem_b=left_upper_clamp,
        min_overlap=0.004,
    )
    ctx.expect_overlap(
        inner_post,
        inner_post,
        axes="xz",
        elem_a=rear_bolt,
        elem_b=right_upper_clamp,
        min_overlap=0.004,
    )
    ctx.expect_contact(
        cable_lever,
        outer_sleeve,
        elem_a=pivot_barrel,
        elem_b=bracket_left,
    )
    ctx.expect_contact(
        cable_lever,
        outer_sleeve,
        elem_a=pivot_barrel,
        elem_b=bracket_right,
    )
    ctx.expect_gap(
        cable_lever,
        outer_sleeve,
        axis="x",
        positive_elem=lever_arm,
        negative_elem=bracket_body,
        min_gap=0.001,
    )
    ctx.expect_contact(
        cable_lever,
        cable_lever,
        elem_a=exposed_cable,
        elem_b=cable_nipple,
    )
    ctx.expect_gap(
        outer_sleeve,
        cable_lever,
        axis="z",
        positive_elem=cable_stop,
        negative_elem=exposed_cable,
        min_gap=0.0,
        max_gap=0.016,
    )

    with ctx.pose({inner_slide: 0.060}):
        ctx.expect_origin_distance(inner_post, outer_sleeve, axes="xy", max_dist=0.0015)
        ctx.expect_gap(
            outer_sleeve,
            inner_post,
            axis="y",
            positive_elem=bushing_front,
            negative_elem=stanchion,
            max_gap=0.0004,
            max_penetration=0.0003,
        )
        ctx.expect_gap(
            outer_sleeve,
            inner_post,
            axis="x",
            positive_elem=bushing_right,
            negative_elem=stanchion,
            max_gap=0.0004,
            max_penetration=0.0003,
        )
        ctx.expect_overlap(
            inner_post,
            outer_sleeve,
            axes="xy",
            elem_a=guide_collar,
            elem_b=boot_lower,
            min_overlap=0.004,
        )

    with ctx.pose({lever_joint: -0.45}):
        ctx.expect_gap(
            cable_lever,
            outer_sleeve,
            axis="x",
            positive_elem=lever_arm,
            negative_elem=outer_body_shell,
            min_gap=0.004,
        )
        ctx.expect_contact(
            cable_lever,
            outer_sleeve,
            elem_a=pivot_barrel,
            elem_b=bracket_left,
        )

    with ctx.pose({inner_slide: 0.060, lever_joint: -0.45}):
        ctx.expect_origin_distance(inner_post, outer_sleeve, axes="xy", max_dist=0.0015)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
