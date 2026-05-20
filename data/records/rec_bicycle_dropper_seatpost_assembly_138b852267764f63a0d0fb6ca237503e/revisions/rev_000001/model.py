from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


OUTER_TUBE_LENGTH = 0.285
DROPPER_TRAVEL = 0.170
BATTERY_CAP_OPEN = math.radians(75.0)
SADDLE_TILT_LIMIT = math.radians(12.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


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


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _saddle_section(
    x: float,
    width: float,
    height: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    half = width * 0.5
    return [
        (x, -half, z_center - 0.15 * height),
        (x, -0.34 * width, z_center + 0.18 * height),
        (x, -0.14 * width, z_center + 0.42 * height),
        (x, 0.00, z_center + 0.52 * height),
        (x, 0.14 * width, z_center + 0.42 * height),
        (x, 0.34 * width, z_center + 0.18 * height),
        (x, half, z_center - 0.15 * height),
        (x, 0.30 * width, z_center - 0.46 * height),
        (x, 0.00, z_center - 0.58 * height),
        (x, -0.30 * width, z_center - 0.46 * height),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    polished_alloy = model.material("polished_alloy", rgba=(0.78, 0.80, 0.83, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.47, 0.49, 0.52, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    carbon_shell = model.material("carbon_shell", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))

    outer_tube = model.part("outer_tube")
    outer_shell = LatheGeometry.from_shell_profiles(
        [(0.0170, 0.000), (0.0170, 0.238), (0.0195, 0.255), (0.0200, 0.273)],
        [(0.0152, 0.000), (0.0152, 0.236), (0.0174, 0.255), (0.0186, 0.273)],
        segments=88,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    outer_tube.visual(
        _save_mesh("outer_tube_shell", outer_shell),
        material=anodized_black,
        name="outer_shell",
    )
    seal_head = LatheGeometry.from_shell_profiles(
        [(0.0205, 0.000), (0.0205, 0.012)],
        [(0.0188, 0.000), (0.0188, 0.012)],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    outer_tube.visual(
        _save_mesh("outer_tube_seal_head", seal_head),
        origin=Origin(xyz=(0.0, 0.0, 0.273)),
        material=dark_metal,
        name="seal_head",
    )
    outer_tube.visual(
        Cylinder(radius=0.0140, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0185, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=seal_gray,
        name="battery_port",
    )
    outer_tube.visual(
        Cylinder(radius=0.0108, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0200, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="battery_port_seal",
    )
    outer_tube.visual(
        Box((0.020, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.0165, 0.218)),
        material=dark_metal,
        name="battery_hinge_pad",
    )
    for x_pos, name in ((-0.007, "battery_hinge_mount_a"), (0.007, "battery_hinge_mount_b")):
        outer_tube.visual(
            Cylinder(radius=0.0026, length=0.006),
            origin=Origin(
                xyz=(x_pos, 0.0180, 0.219),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=dark_metal,
            name=name,
        )
    outer_tube.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, OUTER_TUBE_LENGTH)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TUBE_LENGTH * 0.5)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0143, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=polished_alloy,
        name="shaft",
    )
    inner_post.visual(
        Cylinder(radius=0.0148, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material=dark_metal,
        name="guide_sleeve",
    )
    inner_post.visual(
        Cylinder(radius=0.0205, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_metal,
        name="wiper_collar",
    )
    inner_post.visual(
        Cylinder(radius=0.0185, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_metal,
        name="crown_collar",
    )
    inner_post.visual(
        Box((0.052, 0.028, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=dark_metal,
        name="crown_head",
    )
    inner_post.visual(
        Box((0.068, 0.014, 0.0046)),
        origin=Origin(xyz=(0.0, -0.036, 0.0979)),
        material=seal_gray,
        name="left_cradle",
    )
    inner_post.visual(
        Box((0.068, 0.014, 0.0046)),
        origin=Origin(xyz=(0.0, 0.036, 0.0979)),
        material=seal_gray,
        name="right_cradle",
    )
    inner_post.visual(
        Box((0.026, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, -0.023, 0.086)),
        material=dark_metal,
        name="left_clamp_arm",
    )
    inner_post.visual(
        Box((0.026, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.023, 0.086)),
        material=dark_metal,
        name="right_clamp_arm",
    )
    inner_post.visual(
        Box((0.020, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.018, 0.094)),
        material=dark_metal,
        name="left_cheek",
    )
    inner_post.visual(
        Box((0.020, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.018, 0.094)),
        material=dark_metal,
        name="right_cheek",
    )
    for y_pos, name in ((-0.014, "pivot_knuckle_left"), (0.014, "pivot_knuckle_right")):
        inner_post.visual(
            Cylinder(radius=0.0070, length=0.010),
            origin=Origin(
                xyz=(0.0, y_pos, 0.094),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_metal,
            name=name,
        )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.070, 0.060, 0.390)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
    )

    saddle = model.part("saddle")
    saddle_sections = [
        _saddle_section(0.135, 0.024, 0.018, 0.046),
        _saddle_section(0.085, 0.052, 0.026, 0.043),
        _saddle_section(0.015, 0.084, 0.030, 0.039),
        _saddle_section(-0.070, 0.128, 0.038, 0.043),
        _saddle_section(-0.135, 0.146, 0.034, 0.048),
    ]
    saddle_shell = repair_loft(section_loft(saddle_sections), repair="mesh")
    saddle.visual(
        _save_mesh("saddle_shell", saddle_shell),
        material=carbon_shell,
        name="shell",
    )
    left_rail_path = [
        (-0.115, -0.036, 0.029),
        (-0.085, -0.036, 0.016),
        (-0.040, -0.036, 0.010),
        (0.030, -0.036, 0.010),
        (0.086, -0.036, 0.016),
        (0.120, -0.036, 0.028),
    ]
    right_rail_path = [(x, -y, z) for x, y, z in left_rail_path]
    saddle.visual(
        _save_mesh(
            "saddle_left_rail",
            tube_from_spline_points(
                left_rail_path,
                radius=0.0032,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=rail_steel,
        name="left_rail",
    )
    saddle.visual(
        _save_mesh(
            "saddle_right_rail",
            tube_from_spline_points(
                right_rail_path,
                radius=0.0032,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=rail_steel,
        name="right_rail",
    )
    saddle.visual(
        Cylinder(radius=0.0068, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_barrel",
    )
    saddle.visual(
        Box((0.024, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_metal,
        name="pivot_web",
    )
    saddle.visual(
        Box((0.016, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=dark_metal,
        name="pivot_tower",
    )
    saddle.visual(
        Box((0.060, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, -0.036, 0.031)),
        material=seal_gray,
        name="left_upper_cap",
    )
    saddle.visual(
        Box((0.060, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.036, 0.031)),
        material=seal_gray,
        name="right_upper_cap",
    )
    saddle.visual(
        Box((0.030, 0.084, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        material=seal_gray,
        name="upper_bridge",
    )
    saddle.visual(
        Box((0.020, 0.022, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_metal,
        name="bolt_block",
    )
    saddle.visual(
        Cylinder(radius=0.0036, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=dark_metal,
        name="bolt_shaft",
    )
    saddle.visual(
        Cylinder(radius=0.0070, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_metal,
        name="bolt_head",
    )
    saddle.visual(
        Box((0.020, 0.022, 0.010)),
        origin=Origin(xyz=(0.132, 0.0, 0.050)),
        material=carbon_shell,
        name="nose_tip",
    )
    support_pairs = (
        ((-0.084, -0.036, 0.016), (-0.086, -0.033, 0.032)),
        ((0.000, -0.036, 0.010), (0.005, -0.030, 0.028)),
        ((0.086, -0.036, 0.016), (0.082, -0.030, 0.035)),
        ((-0.084, 0.036, 0.016), (-0.086, 0.033, 0.032)),
        ((0.000, 0.036, 0.010), (0.005, 0.030, 0.028)),
        ((0.086, 0.036, 0.016), (0.082, 0.030, 0.035)),
    )
    for index, (a, b) in enumerate(support_pairs):
        _add_member(
            saddle,
            a,
            b,
            radius=0.0036,
            material=dark_metal,
            name=f"shell_support_{index}",
        )
    saddle.inertial = Inertial.from_geometry(
        Box((0.290, 0.155, 0.085)),
        mass=0.30,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
    )

    battery_cap = model.part("battery_cap")
    battery_cap.visual(
        Cylinder(radius=0.0024, length=0.007),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    battery_cap.visual(
        Box((0.022, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0025, -0.004)),
        material=dark_metal,
        name="hinge_arm",
    )
    battery_cap.visual(
        Cylinder(radius=0.0145, length=0.004),
        origin=Origin(xyz=(0.0, 0.0050, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="cap_disc",
    )
    battery_cap.visual(
        Cylinder(radius=0.0155, length=0.002),
        origin=Origin(xyz=(0.0, 0.0070, -0.015), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="cap_knurl",
    )
    battery_cap.visual(
        Box((0.010, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, 0.0085, -0.026)),
        material=seal_gray,
        name="cap_tab",
    )
    battery_cap.inertial = Inertial.from_geometry(
        Box((0.040, 0.020, 0.040)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.005, -0.015)),
    )

    model.articulation(
        "outer_to_inner_dropper",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, OUTER_TUBE_LENGTH)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=DROPPER_TRAVEL,
        ),
    )
    model.articulation(
        "outer_to_battery_cap",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=battery_cap,
        origin=Origin(xyz=(0.0, 0.0180, 0.219)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=BATTERY_CAP_OPEN,
        ),
    )
    model.articulation(
        "inner_to_saddle_tilt",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.094)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=-SADDLE_TILT_LIMIT,
            upper=SADDLE_TILT_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    saddle = object_model.get_part("saddle")
    battery_cap = object_model.get_part("battery_cap")
    dropper = object_model.get_articulation("outer_to_inner_dropper")
    battery_hinge = object_model.get_articulation("outer_to_battery_cap")
    saddle_tilt = object_model.get_articulation("inner_to_saddle_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        inner_post,
        outer_tube,
        elem_a="wiper_collar",
        elem_b="seal_head",
        name="inner post seats on the outer tube wiper head at rest",
    )
    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem="shaft",
        outer_elem="outer_shell",
        margin=0.003,
        name="inner shaft stays centered in the round outer tube at rest",
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="z",
        elem_a="shaft",
        elem_b="outer_shell",
        min_overlap=0.220,
        name="lowered post retains deep insertion in the outer tube",
    )

    crown_rest = ctx.part_element_world_aabb(inner_post, elem="crown_head")
    with ctx.pose({dropper: DROPPER_TRAVEL}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="shaft",
            outer_elem="outer_shell",
            margin=0.003,
            name="extended shaft remains coaxial inside the outer tube",
        )
        ctx.expect_overlap(
            inner_post,
            outer_tube,
            axes="z",
            elem_a="shaft",
            elem_b="outer_shell",
            min_overlap=0.075,
            name="full extension still leaves retained insertion in the outer tube",
        )
        crown_extended = ctx.part_element_world_aabb(inner_post, elem="crown_head")
    ctx.check(
        "dropper travel raises the crown",
        crown_rest is not None
        and crown_extended is not None
        and crown_extended[1][2] > crown_rest[1][2] + 0.150,
        details=f"rest={crown_rest}, extended={crown_extended}",
    )

    with ctx.pose({battery_hinge: 0.0}):
        ctx.expect_gap(
            battery_cap,
            outer_tube,
            axis="y",
            positive_elem="cap_disc",
            negative_elem="battery_port",
            max_gap=0.002,
            max_penetration=0.0,
            name="battery cap closes flush over the side port",
        )
        ctx.expect_overlap(
            battery_cap,
            outer_tube,
            axes="xz",
            elem_a="cap_disc",
            elem_b="battery_port",
            min_overlap=0.020,
            name="battery cap covers the battery compartment opening",
        )
    cap_closed = ctx.part_element_world_aabb(battery_cap, elem="cap_disc")
    with ctx.pose({battery_hinge: BATTERY_CAP_OPEN}):
        cap_open = ctx.part_element_world_aabb(battery_cap, elem="cap_disc")
    ctx.check(
        "battery cap swings outward on its side hinge",
        cap_closed is not None
        and cap_open is not None
        and cap_open[1][1] > cap_closed[1][1] + 0.018,
        details=f"closed={cap_closed}, open={cap_open}",
    )

    ctx.expect_contact(
        saddle,
        inner_post,
        elem_a="pivot_barrel",
        elem_b="pivot_knuckle_left",
        name="saddle pivot barrel is carried by the clamp knuckles",
    )
    ctx.expect_overlap(
        saddle,
        inner_post,
        axes="xy",
        elem_a="upper_bridge",
        elem_b="crown_head",
        min_overlap=0.026,
        name="upper clamp bridge stays centered over the crown head",
    )
    ctx.expect_overlap(
        saddle,
        inner_post,
        axes="xy",
        elem_a="left_upper_cap",
        elem_b="left_cradle",
        min_overlap=0.010,
        name="left upper clamp cap sits over the left lower cradle",
    )
    ctx.expect_overlap(
        saddle,
        inner_post,
        axes="xy",
        elem_a="right_upper_cap",
        elem_b="right_cradle",
        min_overlap=0.010,
        name="right upper clamp cap sits over the right lower cradle",
    )
    nose_rest = ctx.part_element_world_aabb(saddle, elem="nose_tip")
    with ctx.pose({saddle_tilt: math.radians(10.0)}):
        nose_up = ctx.part_element_world_aabb(saddle, elem="nose_tip")
    ctx.check(
        "positive saddle tilt raises the nose",
        nose_rest is not None and nose_up is not None and nose_up[1][2] > nose_rest[1][2] + 0.010,
        details=f"rest={nose_rest}, up={nose_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
