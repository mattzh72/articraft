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
)


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


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        (mins[0] + maxs[0]) * 0.5,
        (mins[1] + maxs[1]) * 0.5,
        (mins[2] + maxs[2]) * 0.5,
    )


def _cylindrical_shell_mesh(name: str, outer_radius: float, inner_radius: float, length: float):
    half = length * 0.5
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, -half), (outer_radius, half)],
            [(inner_radius, -half), (inner_radius, half)],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="construction_light_tower_trailer")

    trailer_yellow = model.material("trailer_yellow", rgba=(0.93, 0.75, 0.14, 1.0))
    mast_yellow = model.material("mast_yellow", rgba=(0.95, 0.79, 0.18, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.23, 0.25, 1.0))
    near_black = model.material("near_black", rgba=(0.09, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.69, 1.0))
    silver = model.material("silver", rgba=(0.78, 0.80, 0.82, 1.0))
    lamp_glass = model.material("lamp_glass", rgba=(0.82, 0.88, 0.92, 0.55))

    trailer_frame = model.part("trailer_frame")
    trailer_frame.visual(
        Box((2.10, 0.08, 0.12)),
        origin=Origin(xyz=(0.00, 0.42, 0.48)),
        material=dark_grey,
        name="left_frame_rail",
    )
    trailer_frame.visual(
        Box((2.10, 0.08, 0.12)),
        origin=Origin(xyz=(0.00, -0.42, 0.48)),
        material=dark_grey,
        name="right_frame_rail",
    )
    trailer_frame.visual(
        Box((0.12, 0.92, 0.12)),
        origin=Origin(xyz=(0.99, 0.0, 0.48)),
        material=dark_grey,
        name="front_crossmember",
    )
    trailer_frame.visual(
        Box((0.12, 0.92, 0.12)),
        origin=Origin(xyz=(-1.00, 0.0, 0.48)),
        material=dark_grey,
        name="rear_crossmember",
    )
    trailer_frame.visual(
        Box((0.12, 0.92, 0.12)),
        origin=Origin(xyz=(-0.15, 0.0, 0.48)),
        material=dark_grey,
        name="axle_crossmember",
    )
    trailer_frame.visual(
        Box((1.22, 0.96, 0.06)),
        origin=Origin(xyz=(0.10, 0.0, 0.57)),
        material=trailer_yellow,
        name="deck_plate",
    )
    trailer_frame.visual(
        Box((1.06, 0.74, 0.88)),
        origin=Origin(xyz=(0.20, 0.0, 1.01)),
        material=trailer_yellow,
        name="generator_housing",
    )
    trailer_frame.visual(
        Box((1.10, 0.78, 0.03)),
        origin=Origin(xyz=(0.20, 0.0, 1.465)),
        material=dark_grey,
        name="housing_roof",
    )
    trailer_frame.visual(
        Box((0.30, 0.34, 0.42)),
        origin=Origin(xyz=(0.78, 0.0, 0.81)),
        material=trailer_yellow,
        name="control_box",
    )
    trailer_frame.visual(
        Cylinder(radius=0.028, length=0.42),
        origin=Origin(xyz=(0.52, -0.28, 1.69)),
        material=near_black,
        name="exhaust_stack",
    )
    trailer_frame.visual(
        Cylinder(radius=0.05, length=1.04),
        origin=Origin(
            xyz=(-0.15, 0.0, 0.34),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="axle_tube",
    )
    trailer_frame.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(
            xyz=(-0.15, 0.57, 0.34),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_stub_axle",
    )
    trailer_frame.visual(
        Cylinder(radius=0.045, length=0.10),
        origin=Origin(
            xyz=(-0.15, -0.57, 0.34),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_stub_axle",
    )
    trailer_frame.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(-0.15, 0.34, 0.39)),
        material=dark_grey,
        name="left_spring_mount",
    )
    trailer_frame.visual(
        Box((0.18, 0.16, 0.10)),
        origin=Origin(xyz=(-0.15, -0.34, 0.39)),
        material=dark_grey,
        name="right_spring_mount",
    )
    _add_member(
        trailer_frame,
        (0.96, 0.22, 0.48),
        (1.80, 0.00, 0.42),
        radius=0.04,
        material=dark_grey,
        name="left_drawbar_rail",
    )
    _add_member(
        trailer_frame,
        (0.96, -0.22, 0.48),
        (1.80, 0.00, 0.42),
        radius=0.04,
        material=dark_grey,
        name="right_drawbar_rail",
    )
    trailer_frame.visual(
        Box((0.18, 0.08, 0.08)),
        origin=Origin(xyz=(1.88, 0.0, 0.42)),
        material=steel,
        name="tow_coupler",
    )
    trailer_frame.visual(
        Box((0.18, 0.12, 0.10)),
        origin=Origin(xyz=(1.44, 0.0, 0.45)),
        material=dark_grey,
        name="jack_clamp",
    )
    trailer_frame.visual(
        Cylinder(radius=0.045, length=0.42),
        origin=Origin(xyz=(1.44, 0.0, 0.21)),
        material=steel,
        name="landing_jack",
    )
    trailer_frame.visual(
        Box((0.18, 0.18, 0.03)),
        origin=Origin(xyz=(1.44, 0.0, 0.015)),
        material=steel,
        name="jack_pad",
    )
    trailer_frame.visual(
        _cylindrical_shell_mesh("lower_socket_shell", 0.10, 0.082, 0.36),
        origin=Origin(xyz=(-0.78, 0.0, 0.38)),
        material=dark_grey,
        name="lower_socket",
    )
    trailer_frame.visual(
        _cylindrical_shell_mesh("outer_sleeve_shell", 0.11, 0.082, 2.02),
        origin=Origin(xyz=(-0.78, 0.0, 1.57)),
        material=mast_yellow,
        name="outer_sleeve",
    )
    trailer_frame.visual(
        Box((0.16, 0.26, 0.12)),
        origin=Origin(xyz=(-0.94, 0.0, 2.64)),
        material=dark_grey,
        name="sleeve_head_saddle",
    )
    trailer_frame.visual(
        Box((0.01, 0.16, 0.12)),
        origin=Origin(xyz=(-0.86, 0.0, 2.64)),
        material=steel,
        name="mast_guide_pad",
    )
    trailer_frame.visual(
        Box((0.10, 0.18, 0.28)),
        origin=Origin(xyz=(-0.94, 0.0, 0.74)),
        material=dark_grey,
        name="mast_winch_box",
    )
    trailer_frame.visual(
        Box((0.14, 0.22, 2.34)),
        origin=Origin(xyz=(-0.95, 0.0, 1.53)),
        material=dark_grey,
        name="mast_backbone",
    )
    trailer_frame.visual(
        Cylinder(radius=0.035, length=0.42),
        origin=Origin(xyz=(-0.98, 0.34, 0.21)),
        material=steel,
        name="left_rear_stabilizer",
    )
    trailer_frame.visual(
        Cylinder(radius=0.035, length=0.42),
        origin=Origin(xyz=(-0.98, -0.34, 0.21)),
        material=steel,
        name="right_rear_stabilizer",
    )
    trailer_frame.visual(
        Box((0.14, 0.14, 0.02)),
        origin=Origin(xyz=(-0.98, 0.34, 0.01)),
        material=steel,
        name="left_rear_pad",
    )
    trailer_frame.visual(
        Box((0.14, 0.14, 0.02)),
        origin=Origin(xyz=(-0.98, -0.34, 0.01)),
        material=steel,
        name="right_rear_pad",
    )
    trailer_frame.inertial = Inertial.from_geometry(
        Box((3.10, 1.60, 2.80)),
        mass=1150.0,
        origin=Origin(xyz=(0.10, 0.0, 1.40)),
    )

    left_wheel = model.part("left_wheel")
    left_wheel.visual(
        Cylinder(radius=0.34, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=near_black,
        name="tire",
    )
    left_wheel.visual(
        Cylinder(radius=0.23, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rim",
    )
    left_wheel.visual(
        Cylinder(radius=0.08, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.16),
        mass=28.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_wheel = model.part("right_wheel")
    right_wheel.visual(
        Cylinder(radius=0.34, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=near_black,
        name="tire",
    )
    right_wheel.visual(
        Cylinder(radius=0.23, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rim",
    )
    right_wheel.visual(
        Cylinder(radius=0.08, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hub",
    )
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.16),
        mass=28.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    inner_mast = model.part("inner_mast")
    inner_mast.visual(
        Cylinder(radius=0.075, length=6.10),
        origin=Origin(xyz=(0.0, 0.0, 0.70)),
        material=steel,
        name="inner_tube",
    )
    inner_mast.visual(
        Cylinder(radius=0.095, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 3.81)),
        material=dark_grey,
        name="mast_top_cap",
    )
    inner_mast.visual(
        Box((0.14, 0.18, 0.18)),
        origin=Origin(xyz=(-0.12, 0.0, 3.95)),
        material=dark_grey,
        name="yoke_block",
    )
    inner_mast.visual(
        Box((0.10, 1.06, 0.10)),
        origin=Origin(xyz=(-0.08, 0.0, 3.95)),
        material=dark_grey,
        name="yoke_crossbeam",
    )
    inner_mast.visual(
        Box((0.08, 0.05, 0.24)),
        origin=Origin(xyz=(0.12, 0.55, 4.02)),
        material=dark_grey,
        name="left_yoke_arm",
    )
    inner_mast.visual(
        Box((0.08, 0.05, 0.24)),
        origin=Origin(xyz=(0.12, -0.55, 4.02)),
        material=dark_grey,
        name="right_yoke_arm",
    )
    _add_member(
        inner_mast,
        (-0.11, 0.14, 3.95),
        (0.07, 0.55, 3.92),
        radius=0.025,
        material=dark_grey,
        name="left_yoke_brace",
    )
    _add_member(
        inner_mast,
        (-0.11, -0.14, 3.95),
        (0.07, -0.55, 3.92),
        radius=0.025,
        material=dark_grey,
        name="right_yoke_brace",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=6.30),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
    )

    lamp_cluster = model.part("lamp_cluster")
    lamp_cluster.visual(
        Cylinder(radius=0.03, length=1.00),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="trunnion_shaft",
    )
    lamp_cluster.visual(
        Box((0.10, 0.94, 0.44)),
        origin=Origin(xyz=(0.03, 0.0, -0.12)),
        material=dark_grey,
        name="back_frame",
    )
    lamp_cluster.visual(
        Box((0.12, 0.04, 0.36)),
        origin=Origin(xyz=(0.02, 0.48, -0.12)),
        material=dark_grey,
        name="left_side_plate",
    )
    lamp_cluster.visual(
        Box((0.12, 0.04, 0.36)),
        origin=Origin(xyz=(0.02, -0.48, -0.12)),
        material=dark_grey,
        name="right_side_plate",
    )
    lamp_cluster.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(
            xyz=(0.0, 0.495, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion_boss",
    )
    lamp_cluster.visual(
        Cylinder(radius=0.045, length=0.06),
        origin=Origin(
            xyz=(0.0, -0.495, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion_boss",
    )
    lamp_cluster.visual(
        Box((0.10, 0.94, 0.05)),
        origin=Origin(xyz=(0.09, 0.0, 0.105)),
        material=dark_grey,
        name="top_visor",
    )

    lamp_positions = (
        ("lamp_head_upper_left", 0.24, 0.24, -0.04),
        ("lamp_head_lower_left", 0.24, 0.24, -0.26),
        ("lamp_head_upper_right", 0.24, -0.24, -0.04),
        ("lamp_head_lower_right", 0.24, -0.24, -0.26),
    )
    for lamp_name, lamp_x, lamp_y, lamp_z in lamp_positions:
        lamp_cluster.visual(
            Box((0.16, 0.20, 0.16)),
            origin=Origin(xyz=(lamp_x, lamp_y, lamp_z)),
            material=near_black,
            name=lamp_name,
        )
        lamp_cluster.visual(
            Box((0.01, 0.18, 0.14)),
            origin=Origin(xyz=(lamp_x + 0.085, lamp_y, lamp_z)),
            material=lamp_glass,
            name=f"{lamp_name}_lens",
        )
        lamp_cluster.visual(
            Box((0.14, 0.04, 0.10)),
            origin=Origin(xyz=(0.15, lamp_y, lamp_z)),
            material=dark_grey,
            name=f"{lamp_name}_mount",
        )
    lamp_cluster.inertial = Inertial.from_geometry(
        Box((0.42, 1.02, 0.62)),
        mass=55.0,
        origin=Origin(xyz=(0.16, 0.0, -0.12)),
    )

    model.articulation(
        "trailer_to_left_wheel",
        ArticulationType.FIXED,
        parent=trailer_frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.15, 0.70, 0.34)),
    )
    model.articulation(
        "trailer_to_right_wheel",
        ArticulationType.FIXED,
        parent=trailer_frame,
        child=right_wheel,
        origin=Origin(xyz=(-0.15, -0.70, 0.34)),
    )
    model.articulation(
        "trailer_to_inner_mast",
        ArticulationType.PRISMATIC,
        parent=trailer_frame,
        child=inner_mast,
        origin=Origin(xyz=(-0.78, 0.0, 2.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.25,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "mast_to_lamp_cluster",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=lamp_cluster,
        origin=Origin(xyz=(0.14, 0.0, 4.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.8,
            lower=-0.55,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    trailer_frame = object_model.get_part("trailer_frame")
    inner_mast = object_model.get_part("inner_mast")
    lamp_cluster = object_model.get_part("lamp_cluster")
    mast_slide = object_model.get_articulation("trailer_to_inner_mast")
    lamp_tilt = object_model.get_articulation("mast_to_lamp_cluster")

    mast_upper = 2.05
    lamp_upper = 0.95

    ctx.expect_within(
        inner_mast,
        trailer_frame,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.015,
        name="mast stays centered inside the sleeve at rest",
    )
    ctx.expect_overlap(
        inner_mast,
        trailer_frame,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=1.95,
        name="collapsed mast remains deeply inserted in the sleeve",
    )
    ctx.expect_contact(
        inner_mast,
        trailer_frame,
        elem_a="inner_tube",
        elem_b="mast_guide_pad",
        contact_tol=1e-6,
        name="mast tube bears against the trailer guide pad",
    )

    rest_mast_position = ctx.part_world_position(inner_mast)
    with ctx.pose({mast_slide: mast_upper}):
        ctx.expect_within(
            inner_mast,
            trailer_frame,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.015,
            name="extended mast stays centered inside the sleeve",
        )
        ctx.expect_overlap(
            inner_mast,
            trailer_frame,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.29,
            name="extended mast still retains insertion in the sleeve",
        )
        extended_mast_position = ctx.part_world_position(inner_mast)

    ctx.check(
        "mast extends upward from the trailer socket",
        rest_mast_position is not None
        and extended_mast_position is not None
        and extended_mast_position[2] > rest_mast_position[2] + 2.0,
        details=f"rest={rest_mast_position}, extended={extended_mast_position}",
    )

    rest_lamp_center = _aabb_center(
        ctx.part_element_world_aabb(lamp_cluster, elem="lamp_head_upper_left")
    )
    with ctx.pose({lamp_tilt: lamp_upper}):
        raised_lamp_center = _aabb_center(
            ctx.part_element_world_aabb(lamp_cluster, elem="lamp_head_upper_left")
        )

    ctx.check(
        "lamp cluster tilts upward at the mast yoke",
        rest_lamp_center is not None
        and raised_lamp_center is not None
        and raised_lamp_center[2] > rest_lamp_center[2] + 0.10,
        details=f"rest={rest_lamp_center}, raised={raised_lamp_center}",
    )

    ctx.expect_contact(
        lamp_cluster,
        inner_mast,
        elem_a="left_trunnion_boss",
        elem_b="left_yoke_arm",
        contact_tol=1e-6,
        name="left trunnion boss seats against the left yoke arm",
    )
    ctx.expect_contact(
        lamp_cluster,
        inner_mast,
        elem_a="right_trunnion_boss",
        elem_b="right_yoke_arm",
        contact_tol=1e-6,
        name="right trunnion boss seats against the right yoke arm",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
