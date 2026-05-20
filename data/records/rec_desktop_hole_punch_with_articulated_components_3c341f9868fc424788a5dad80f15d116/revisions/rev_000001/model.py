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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_punch")

    steel = model.material("steel", rgba=(0.69, 0.71, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.45, 0.48, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.300, 0.125, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=steel,
        name="foot",
    )
    base.visual(
        Box((0.270, 0.004, 0.028)),
        origin=Origin(xyz=(0.004, -0.0605, 0.014)),
        material=steel,
        name="side_wall_0",
    )
    base.visual(
        Box((0.100, 0.004, 0.028)),
        origin=Origin(xyz=(-0.081, 0.0605, 0.014)),
        material=steel,
        name="side_wall_1",
    )
    base.visual(
        Box((0.085, 0.004, 0.028)),
        origin=Origin(xyz=(0.108, 0.0605, 0.014)),
        material=steel,
        name="side_wall_2",
    )
    base.visual(
        Box((0.004, 0.117, 0.026)),
        origin=Origin(xyz=(-0.128, 0.0, 0.013)),
        material=steel,
        name="rear_wall",
    )
    base.visual(
        Box((0.010, 0.117, 0.016)),
        origin=Origin(xyz=(0.145, 0.0, 0.008)),
        material=steel,
        name="front_lip",
    )
    base.visual(
        Box((0.190, 0.112, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, 0.030)),
        material=steel,
        name="punch_deck",
    )
    for index, offset_y in enumerate((-0.045, 0.045)):
        base.visual(
            Box((0.175, 0.010, 0.024)),
            origin=Origin(xyz=(0.012, offset_y, 0.018)),
            material=steel,
            name=f"support_rib_{index}",
        )
    base.visual(
        Box((0.080, 0.112, 0.022)),
        origin=Origin(xyz=(-0.090, 0.0, 0.041)),
        material=dark_steel,
        name="rear_housing",
    )
    for index, offset_y in enumerate((-0.040, 0.040)):
        base.visual(
            Cylinder(radius=0.0115, length=0.018),
            origin=Origin(xyz=(0.020, offset_y, 0.041), rpy=(0.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"die_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.300, 0.125, 0.063)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
    )

    handle = model.part("handle")

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x_pos: float,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        return [
            (x_pos, y, z + z_center)
            for z, y in rounded_rect_profile(height, width, radius)
        ]

    handle_geom = section_loft(
        [
            yz_section(0.090, 0.026, 0.010, 0.000, 0.005),
            yz_section(0.108, 0.032, 0.012, 0.080, 0.006),
            yz_section(0.120, 0.030, 0.012, 0.165, 0.002),
            yz_section(0.094, 0.018, 0.008, 0.245, -0.004),
        ]
    )
    handle.visual(
        mesh_from_geometry(handle_geom, "handle_shell"),
        material=satin_black,
        name="shell",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.106),
        origin=Origin(xyz=(0.000, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    handle.visual(
        Box((0.038, 0.102, 0.010)),
        origin=Origin(xyz=(0.112, 0.0, -0.003)),
        material=dark_steel,
        name="carrier",
    )
    for index, offset_y in enumerate((-0.040, 0.040)):
        handle.visual(
            Cylinder(radius=0.009, length=0.014),
            origin=Origin(xyz=(0.112, offset_y, -0.018)),
            material=dark_steel,
            name=f"punch_{index}",
        )
    handle.visual(
        Box((0.090, 0.064, 0.010)),
        origin=Origin(xyz=(0.175, 0.0, -0.010)),
        material=dark_steel,
        name="spine",
    )
    handle.visual(
        Box((0.024, 0.092, 0.008)),
        origin=Origin(xyz=(0.228, 0.0, -0.004)),
        material=dark_steel,
        name="nose",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.255, 0.120, 0.052)),
        mass=0.9,
        origin=Origin(xyz=(0.128, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.108, 0.0, 0.064)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    guide = model.part("guide")
    guide.visual(
        Box((0.070, 0.150, 0.0025)),
        origin=Origin(xyz=(0.0, 0.020, 0.00125)),
        material=steel,
        name="bar",
    )
    guide.visual(
        Box((0.052, 0.008, 0.013)),
        origin=Origin(xyz=(0.0, 0.054, 0.009)),
        material=steel,
        name="stop",
    )
    guide.visual(
        Box((0.026, 0.016, 0.004)),
        origin=Origin(xyz=(0.0, 0.065, 0.0045)),
        material=satin_black,
        name="grip",
    )
    guide.inertial = Inertial.from_geometry(
        Box((0.070, 0.150, 0.016)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.020, 0.008)),
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=guide,
        origin=Origin(xyz=(0.020, 0.0, 0.0115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.15,
            lower=0.0,
            upper=0.055,
        ),
    )

    lock = model.part("lock")
    lock.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot",
    )
    lock.visual(
        Box((0.022, 0.030, 0.004)),
        origin=Origin(xyz=(-0.011, 0.0, 0.002)),
        material=dark_steel,
        name="arm",
    )
    lock.visual(
        Box((0.010, 0.020, 0.010)),
        origin=Origin(xyz=(-0.021, 0.0, 0.007)),
        material=satin_black,
        name="tab",
    )
    lock.inertial = Inertial.from_geometry(
        Box((0.032, 0.030, 0.012)),
        mass=0.03,
        origin=Origin(xyz=(-0.012, 0.0, 0.006)),
    )

    model.articulation(
        "base_to_lock",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lock,
        origin=Origin(xyz=(-0.116, 0.0, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=math.radians(-35.0),
            upper=math.radians(55.0),
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

    handle = object_model.get_part("handle")
    base = object_model.get_part("base")
    hinge = object_model.get_articulation("base_to_handle")
    guide = object_model.get_part("guide")
    guide_slide = object_model.get_articulation("base_to_guide")
    lock = object_model.get_part("lock")
    lock_joint = object_model.get_articulation("base_to_lock")

    ctx.expect_gap(
        handle,
        base,
        axis="z",
        max_gap=0.025,
        max_penetration=0.0,
        positive_elem="carrier",
        negative_elem="punch_deck",
        name="handle sits just above punch deck",
    )
    ctx.expect_gap(
        base,
        guide,
        axis="z",
        min_gap=0.0005,
        max_gap=0.004,
        positive_elem="punch_deck",
        negative_elem="stop",
        name="guide stores below punch line",
    )
    ctx.expect_gap(
        guide,
        base,
        axis="z",
        min_gap=0.004,
        max_gap=0.007,
        positive_elem="bar",
        negative_elem="foot",
        name="guide rides above base foot",
    )
    ctx.expect_gap(
        lock,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem="pivot",
        negative_elem="rear_housing",
        name="lock pivot sits on rear housing",
    )

    rest_front = ctx.part_element_world_aabb(handle, elem="nose")
    rest_grip = ctx.part_element_world_aabb(guide, elem="grip")
    rest_tab = ctx.part_element_world_aabb(lock, elem="tab")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        opened_front = ctx.part_element_world_aabb(handle, elem="nose")
    with ctx.pose({guide_slide: guide_slide.motion_limits.upper}):
        extended_grip = ctx.part_element_world_aabb(guide, elem="grip")
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        raised_tab = ctx.part_element_world_aabb(lock, elem="tab")

    ctx.check(
        "handle opens upward",
        rest_front is not None
        and opened_front is not None
        and opened_front[1][2] > rest_front[1][2] + 0.08,
        details=f"rest={rest_front}, opened={opened_front}",
    )
    ctx.check(
        "guide slides outward",
        rest_grip is not None
        and extended_grip is not None
        and extended_grip[1][1] > rest_grip[1][1] + 0.045,
        details=f"rest={rest_grip}, extended={extended_grip}",
    )
    ctx.check(
        "lock tab lifts when rotated",
        rest_tab is not None
        and raised_tab is not None
        and raised_tab[1][2] > rest_tab[1][2] + 0.015,
        details=f"rest={rest_tab}, raised={raised_tab}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
