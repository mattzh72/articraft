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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    concrete = model.material("concrete", rgba=(0.68, 0.68, 0.66, 1.0))
    tower_gray = model.material("tower_gray", rgba=(0.43, 0.46, 0.50, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.76, 0.79, 0.82, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.80, 0.90, 0.98, 0.35))

    support = model.part("support")
    support.visual(
        Box((1.70, 1.70, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="foundation",
    )
    support.visual(
        Box((1.00, 1.00, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=tower_gray,
        name="equipment_pedestal",
    )
    support.visual(
        Box((0.40, 0.02, 0.52)),
        origin=Origin(xyz=(0.0, 0.51, 0.54)),
        material=machinery_gray,
        name="access_door",
    )
    support.visual(
        Cylinder(radius=0.26, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=machinery_gray,
        name="mast_collar",
    )
    support.visual(
        Cylinder(radius=0.18, length=1.85),
        origin=Origin(xyz=(0.0, 0.0, 2.025)),
        material=tower_gray,
        name="mast_tube",
    )
    support.visual(
        Box((0.62, 0.62, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 2.99)),
        material=machinery_gray,
        name="top_platform",
    )
    support.visual(
        Cylinder(radius=0.24, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 3.17)),
        material=trim_dark,
        name="pan_bearing_housing",
    )
    support.inertial = Inertial.from_geometry(
        Box((1.70, 1.70, 3.31)),
        mass=1400.0,
        origin=Origin(xyz=(0.0, 0.0, 1.655)),
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.15, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=trim_dark,
        name="turntable_drum",
    )
    pan_yoke.visual(
        Cylinder(radius=0.23, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=machinery_gray,
        name="turntable_plate",
    )
    pan_yoke.visual(
        Box((0.22, 0.68, 0.18)),
        origin=Origin(xyz=(0.02, 0.0, 0.36)),
        material=machinery_gray,
        name="yoke_saddle",
    )
    pan_yoke.visual(
        Box((0.10, 0.08, 0.78)),
        origin=Origin(xyz=(0.08, 0.34, 0.84)),
        material=machinery_gray,
        name="left_arm",
    )
    pan_yoke.visual(
        Box((0.10, 0.08, 0.78)),
        origin=Origin(xyz=(0.08, -0.34, 0.84)),
        material=machinery_gray,
        name="right_arm",
    )
    pan_yoke.visual(
        Cylinder(radius=0.05, length=0.76),
        origin=Origin(xyz=(0.08, 0.0, 1.23), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machinery_gray,
        name="top_crosshead",
    )
    pan_yoke.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.08, 0.33, 0.74), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_clamp",
    )
    pan_yoke.visual(
        Cylinder(radius=0.07, length=0.06),
        origin=Origin(xyz=(0.08, -0.33, 0.74), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_clamp",
    )
    pan_yoke.visual(
        Box((0.16, 0.22, 0.16)),
        origin=Origin(xyz=(-0.08, 0.0, 0.39)),
        material=trim_dark,
        name="pan_drive_box",
    )
    pan_yoke.inertial = Inertial.from_geometry(
        Box((0.46, 0.82, 1.28)),
        mass=180.0,
        origin=Origin(xyz=(0.03, 0.0, 0.64)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="trunnion_hub",
    )
    lamp_head.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.02, 0.27, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="left_shoulder",
    )
    lamp_head.visual(
        Cylinder(radius=0.075, length=0.06),
        origin=Origin(xyz=(0.02, -0.27, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="right_shoulder",
    )
    lamp_head.visual(
        Cylinder(radius=0.24, length=0.54),
        origin=Origin(xyz=(0.20, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_gray,
        name="main_barrel",
    )
    lamp_head.visual(
        Cylinder(radius=0.29, length=0.08),
        origin=Origin(xyz=(0.51, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.26, length=0.012),
        origin=Origin(xyz=(0.556, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.14, length=0.18),
        origin=Origin(xyz=(-0.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="rear_housing",
    )
    lamp_head.visual(
        Cylinder(radius=0.10, length=0.05),
        origin=Origin(xyz=(-0.275, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="rear_cap",
    )
    lamp_head.visual(
        Box((0.20, 0.22, 0.10)),
        origin=Origin(xyz=(-0.08, 0.0, -0.19)),
        material=trim_dark,
        name="ballast_box",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.87, 0.58, 0.58)),
        mass=90.0,
        origin=Origin(xyz=(0.13, 0.0, 0.0)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=support,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 3.31)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.7,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.08, 0.0, 0.74)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=400.0,
            velocity=0.8,
            lower=math.radians(-45.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    pan_yoke = object_model.get_part("pan_yoke")
    lamp_head = object_model.get_part("lamp_head")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.expect_gap(
        pan_yoke,
        support,
        axis="z",
        positive_elem="turntable_drum",
        negative_elem="pan_bearing_housing",
        min_gap=0.0,
        max_gap=0.001,
        name="turntable sits directly on the pan bearing housing",
    )
    ctx.expect_gap(
        pan_yoke,
        lamp_head,
        axis="y",
        positive_elem="left_clamp",
        negative_elem="left_shoulder",
        min_gap=0.0,
        max_gap=0.001,
        name="left lamp shoulder stays seated against the left yoke clamp",
    )
    ctx.expect_gap(
        lamp_head,
        pan_yoke,
        axis="y",
        positive_elem="right_shoulder",
        negative_elem="right_clamp",
        min_gap=0.0,
        max_gap=0.001,
        name="right lamp shoulder stays seated against the right yoke clamp",
    )

    support_aabb = ctx.part_world_aabb(support)
    pan_aabb = ctx.part_world_aabb(pan_yoke)
    support_width = None
    pan_width = None
    if support_aabb is not None:
        support_width = support_aabb[1][0] - support_aabb[0][0]
    if pan_aabb is not None:
        pan_width = pan_aabb[1][0] - pan_aabb[0][0]
    ctx.check(
        "fixed support remains much broader than the pan assembly",
        support_width is not None
        and pan_width is not None
        and support_width > pan_width * 3.0,
        details=f"support_width={support_width}, pan_width={pan_width}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    tilt_upper = tilt_axis.motion_limits.upper if tilt_axis.motion_limits is not None else None
    with ctx.pose({tilt_axis: tilt_upper if tilt_upper is not None else 0.0}):
        raised_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    rest_lens_z = None if rest_lens is None else 0.5 * (rest_lens[0][2] + rest_lens[1][2])
    raised_lens_z = None if raised_lens is None else 0.5 * (raised_lens[0][2] + raised_lens[1][2])
    ctx.check(
        "tilt joint raises the lamp nose",
        rest_lens_z is not None
        and raised_lens_z is not None
        and raised_lens_z > rest_lens_z + 0.12,
        details=f"rest_lens_z={rest_lens_z}, raised_lens_z={raised_lens_z}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    with ctx.pose({pan_axis: math.radians(85.0)}):
        panned_lens = ctx.part_element_world_aabb(lamp_head, elem="front_lens")
    rest_lens_xy = (
        None
        if rest_lens is None
        else (
            0.5 * (rest_lens[0][0] + rest_lens[1][0]),
            0.5 * (rest_lens[0][1] + rest_lens[1][1]),
        )
    )
    panned_lens_xy = (
        None
        if panned_lens is None
        else (
            0.5 * (panned_lens[0][0] + panned_lens[1][0]),
            0.5 * (panned_lens[0][1] + panned_lens[1][1]),
        )
    )
    ctx.check(
        "pan joint swings the lamp around the mast",
        rest_lens_xy is not None
        and panned_lens_xy is not None
        and abs(panned_lens_xy[1]) > abs(rest_lens_xy[1]) + 0.40,
        details=f"rest_lens_xy={rest_lens_xy}, panned_lens_xy={panned_lens_xy}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
