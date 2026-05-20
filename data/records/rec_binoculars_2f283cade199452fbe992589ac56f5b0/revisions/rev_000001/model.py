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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _cylinder_x(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_z(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material,
    name: str | None = None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_optical_half(
    part,
    *,
    y_center: float,
    name_prefix: str,
    body_material,
    armor_material,
    rubber_material,
    glass_material,
):
    part.visual(
        Box((0.086, 0.056, 0.020)),
        origin=Origin(xyz=(-0.012, y_center, -0.010)),
        material=body_material,
        name=f"{name_prefix}_main_housing",
    )
    part.visual(
        Box((0.066, 0.050, 0.022)),
        origin=Origin(xyz=(-0.006, y_center, 0.030)),
        material=armor_material,
        name=f"{name_prefix}_upper_housing",
    )
    _cylinder_x(
        part,
        radius=0.028,
        length=0.152,
        xyz=(0.022, y_center, 0.000),
        material=armor_material,
        name=f"{name_prefix}_objective_barrel",
    )
    _cylinder_x(
        part,
        radius=0.031,
        length=0.010,
        xyz=(0.086, y_center, 0.000),
        material=body_material,
        name=f"{name_prefix}_objective_ring",
    )
    _cylinder_x(
        part,
        radius=0.023,
        length=0.004,
        xyz=(0.091, y_center, 0.000),
        material=glass_material,
        name=f"{name_prefix}_objective_glass",
    )
    _cylinder_x(
        part,
        radius=0.020,
        length=0.036,
        xyz=(-0.069, y_center, 0.006),
        material=body_material,
        name=f"{name_prefix}_eyepiece_turret",
    )
    _cylinder_x(
        part,
        radius=0.012,
        length=0.016,
        xyz=(-0.047, y_center, 0.006),
        material=glass_material,
        name=f"{name_prefix}_ocular_glass",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="image_stabilized_binoculars")

    body_material = model.material("body_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    armor_material = model.material("rubber_armor", rgba=(0.09, 0.10, 0.11, 1.0))
    rubber_material = model.material("eyecup_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    trim_material = model.material("trim_black", rgba=(0.13, 0.14, 0.15, 1.0))
    glass_material = model.material("coated_glass", rgba=(0.42, 0.60, 0.68, 0.42))
    prism_material = model.material("stabilizer_block", rgba=(0.45, 0.53, 0.58, 1.0))

    left_body = model.part("left_optic_body")
    _add_optical_half(
        left_body,
        y_center=-0.038,
        name_prefix="left",
        body_material=body_material,
        armor_material=armor_material,
        rubber_material=rubber_material,
        glass_material=glass_material,
    )
    _cylinder_x(
        left_body,
        radius=0.018,
        length=0.020,
        xyz=(-0.091, -0.038, 0.006),
        material=rubber_material,
        name="left_eyecup",
    )
    left_body.visual(
        Box((0.060, 0.016, 0.004)),
        origin=Origin(xyz=(-0.016, -0.006, 0.038)),
        material=body_material,
        name="stabilizer_roof",
    )
    left_body.visual(
        Box((0.056, 0.004, 0.030)),
        origin=Origin(xyz=(-0.016, -0.012, 0.023)),
        material=body_material,
        name="stabilizer_side_wall",
    )
    left_body.visual(
        Box((0.004, 0.016, 0.030)),
        origin=Origin(xyz=(0.012, -0.006, 0.023)),
        material=body_material,
        name="stabilizer_front_wall",
    )
    left_body.visual(
        Box((0.004, 0.016, 0.030)),
        origin=Origin(xyz=(-0.044, -0.006, 0.023)),
        material=body_material,
        name="stabilizer_rear_wall",
    )
    left_body.visual(
        Box((0.028, 0.012, 0.010)),
        origin=Origin(xyz=(-0.008, -0.006, 0.024)),
        material=body_material,
        name="left_bridge_top",
    )
    left_body.visual(
        Box((0.050, 0.016, 0.014)),
        origin=Origin(xyz=(-0.016, -0.012, -0.012)),
        material=body_material,
        name="left_bridge_bottom",
    )
    _cylinder_z(
        left_body,
        radius=0.012,
        length=0.016,
        xyz=(0.000, 0.000, 0.027),
        material=trim_material,
        name="left_bridge_top_knuckle",
    )
    _cylinder_z(
        left_body,
        radius=0.012,
        length=0.016,
        xyz=(0.000, 0.000, -0.023),
        material=trim_material,
        name="left_bridge_bottom_knuckle",
    )
    _cylinder_z(
        left_body,
        radius=0.006,
        length=0.006,
        xyz=(0.000, 0.000, 0.037),
        material=trim_material,
        name="hinge_top_cap",
    )
    _cylinder_z(
        left_body,
        radius=0.006,
        length=0.006,
        xyz=(0.000, 0.000, -0.033),
        material=trim_material,
        name="hinge_bottom_cap",
    )
    left_body.visual(
        Box((0.018, 0.010, 0.018)),
        origin=Origin(xyz=(-0.074, 0.000, 0.029)),
        material=body_material,
        name="focus_platform",
    )
    left_body.visual(
        Box((0.020, 0.004, 0.006)),
        origin=Origin(xyz=(-0.064, -0.004, 0.025)),
        material=trim_material,
        name="focus_neg_cheek",
    )
    left_body.visual(
        Box((0.020, 0.004, 0.006)),
        origin=Origin(xyz=(-0.064, 0.004, 0.025)),
        material=trim_material,
        name="focus_pos_cheek",
    )
    left_body.visual(
        Box((0.052, 0.006, 0.006)),
        origin=Origin(xyz=(-0.049, -0.002, 0.023)),
        material=body_material,
        name="focus_mount_arm",
    )
    left_body.inertial = Inertial.from_geometry(
        Box((0.200, 0.120, 0.080)),
        mass=0.46,
        origin=Origin(xyz=(-0.010, -0.026, 0.015)),
    )

    right_body = model.part("right_optic_body")
    _add_optical_half(
        right_body,
        y_center=0.044,
        name_prefix="right",
        body_material=body_material,
        armor_material=armor_material,
        rubber_material=rubber_material,
        glass_material=glass_material,
    )
    _cylinder_x(
        right_body,
        radius=0.018,
        length=0.020,
        xyz=(-0.091, 0.044, 0.006),
        material=rubber_material,
        name="right_eyecup",
    )
    right_body.visual(
        Box((0.034, 0.018, 0.020)),
        origin=Origin(xyz=(-0.018, 0.016, 0.002)),
        material=body_material,
        name="right_bridge_spine",
    )
    _cylinder_z(
        right_body,
        radius=0.010,
        length=0.024,
        xyz=(0.000, 0.000, 0.002),
        material=trim_material,
        name="right_bridge_knuckle",
    )
    right_body.inertial = Inertial.from_geometry(
        Box((0.190, 0.120, 0.080)),
        mass=0.40,
        origin=Origin(xyz=(-0.008, 0.028, 0.015)),
    )

    prism_block = model.part("stabilizer_prism_block")
    prism_block.visual(
        Box((0.026, 0.010, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=prism_material,
        name="prism_block",
    )
    prism_block.visual(
        Box((0.006, 0.004, 0.017)),
        origin=Origin(xyz=(-0.010, 0.002, 0.0135)),
        material=trim_material,
        name="top_support",
    )
    prism_block.visual(
        Box((0.008, 0.007, 0.012)),
        origin=Origin(xyz=(-0.010, -0.0085, 0.004)),
        material=trim_material,
        name="side_support",
    )
    prism_block.inertial = Inertial.from_geometry(
        Box((0.028, 0.012, 0.030)),
        mass=0.07,
        origin=Origin(xyz=(-0.002, 0.001, 0.010)),
    )

    focus_wheel = model.part("focus_wheel")
    _cylinder_y(
        focus_wheel,
        radius=0.012,
        length=0.024,
        xyz=(0.000, 0.000, 0.000),
        material=trim_material,
        name="wheel_drum",
    )
    _cylinder_y(
        focus_wheel,
        radius=0.014,
        length=0.002,
        xyz=(0.000, -0.011, 0.000),
        material=trim_material,
        name="wheel_neg_flange",
    )
    _cylinder_y(
        focus_wheel,
        radius=0.014,
        length=0.002,
        xyz=(0.000, 0.011, 0.000),
        material=trim_material,
        name="wheel_pos_flange",
    )
    _cylinder_y(
        focus_wheel,
        radius=0.004,
        length=0.028,
        xyz=(0.000, 0.000, 0.000),
        material=body_material,
        name="wheel_hub",
    )
    for index in range(10):
        angle = (2.0 * math.pi * index) / 10.0
        focus_wheel.visual(
            Box((0.0024, 0.022, 0.0048)),
            origin=Origin(
                xyz=(0.0103 * math.cos(angle), 0.000, 0.0103 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=armor_material,
            name=f"knurl_{index:02d}",
        )
    focus_wheel.inertial = Inertial.from_geometry(
        Box((0.030, 0.024, 0.030)),
        mass=0.03,
    )

    model.articulation(
        "bridge_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.4,
            lower=-0.20,
            upper=0.22,
        ),
    )
    model.articulation(
        "left_body_to_stabilizer",
        ArticulationType.FIXED,
        parent=left_body,
        child=prism_block,
        origin=Origin(xyz=(-0.025, 0.002, 0.014)),
    )
    model.articulation(
        "focus_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=left_body,
        child=focus_wheel,
        origin=Origin(xyz=(-0.056, 0.000, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=18.0),
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

    left_body = object_model.get_part("left_optic_body")
    right_body = object_model.get_part("right_optic_body")
    prism_block = object_model.get_part("stabilizer_prism_block")
    focus_wheel = object_model.get_part("focus_wheel")
    bridge_hinge = object_model.get_articulation("bridge_hinge")
    wheel_spin = object_model.get_articulation("focus_wheel_spin")

    ctx.expect_gap(
        right_body,
        left_body,
        axis="y",
        positive_elem="right_eyecup",
        negative_elem="left_eyecup",
        min_gap=0.020,
        name="eyecups have usable rest spacing",
    )
    ctx.expect_contact(
        prism_block,
        left_body,
        elem_a="top_support",
        elem_b="stabilizer_roof",
        name="stabilizer block is hung from the roof shell",
    )
    ctx.expect_contact(
        prism_block,
        left_body,
        elem_a="side_support",
        elem_b="stabilizer_side_wall",
        name="stabilizer block is laterally braced inside the housing",
    )
    bridge_limits = bridge_hinge.motion_limits
    ctx.check(
        "bridge hinge is a realistic IPD revolute joint",
        bridge_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(bridge_hinge.axis) == (0.0, 0.0, -1.0)
        and bridge_limits is not None
        and bridge_limits.lower is not None
        and bridge_limits.upper is not None
        and bridge_limits.lower < 0.0 < bridge_limits.upper
        and (bridge_limits.upper - bridge_limits.lower) >= 0.35,
        details=(
            f"type={bridge_hinge.articulation_type}, axis={bridge_hinge.axis}, "
            f"limits={bridge_limits}"
        ),
    )

    wheel_limits = wheel_spin.motion_limits
    ctx.check(
        "focus wheel uses continuous rotation",
        wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(wheel_spin.axis) == (0.0, 1.0, 0.0)
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=(
            f"type={wheel_spin.articulation_type}, axis={wheel_spin.axis}, "
            f"limits={wheel_limits}"
        ),
    )

    def _center_y(aabb):
        if aabb is None:
            return None
        return (aabb[0][1] + aabb[1][1]) * 0.5

    with ctx.pose({bridge_hinge: -0.16}):
        closed_right = ctx.part_element_world_aabb(right_body, elem="right_eyecup")
        closed_left = ctx.part_element_world_aabb(left_body, elem="left_eyecup")
    with ctx.pose({bridge_hinge: 0.16}):
        open_right = ctx.part_element_world_aabb(right_body, elem="right_eyecup")
        open_left = ctx.part_element_world_aabb(left_body, elem="left_eyecup")

    closed_right_y = _center_y(closed_right)
    open_right_y = _center_y(open_right)
    closed_gap = None
    open_gap = None
    if closed_right is not None and closed_left is not None:
        closed_gap = closed_right[0][1] - closed_left[1][1]
    if open_right is not None and open_left is not None:
        open_gap = open_right[0][1] - open_left[1][1]

    ctx.check(
        "positive bridge motion opens the eyepiece spacing",
        closed_right_y is not None
        and open_right_y is not None
        and closed_gap is not None
        and open_gap is not None
        and open_right_y > closed_right_y + 0.020
        and open_gap > closed_gap + 0.018,
        details=(
            f"closed_right_y={closed_right_y}, open_right_y={open_right_y}, "
            f"closed_gap={closed_gap}, open_gap={open_gap}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
