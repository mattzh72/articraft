from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_hole_punch")

    body_black = model.material("body_black", rgba=(0.18, 0.19, 0.21, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    handle_coat = model.material("handle_coat", rgba=(0.10, 0.11, 0.12, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.78, 0.79, 0.80, 1.0))
    bin_black = model.material("bin_black", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.22, 0.16)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    body.visual(
        Box((0.34, 0.12, 0.010)),
        origin=Origin(xyz=(0.0, -0.040, 0.005)),
        material=body_black,
        name="bottom_front",
    )
    body.visual(
        Box((0.080, 0.100, 0.010)),
        origin=Origin(xyz=(-0.130, 0.060, 0.005)),
        material=body_black,
        name="bottom_left",
    )
    body.visual(
        Box((0.080, 0.100, 0.010)),
        origin=Origin(xyz=(0.130, 0.060, 0.005)),
        material=body_black,
        name="bottom_right",
    )
    body.visual(
        Box((0.180, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.005)),
        material=body_black,
        name="door_frame_front",
    )
    body.visual(
        Box((0.180, 0.028, 0.010)),
        origin=Origin(xyz=(0.0, 0.096, 0.005)),
        material=body_black,
        name="door_frame_rear",
    )
    body.visual(
        Box((0.014, 0.210, 0.050)),
        origin=Origin(xyz=(-0.163, 0.000, 0.035)),
        material=body_black,
        name="side_left",
    )
    body.visual(
        Box((0.014, 0.210, 0.050)),
        origin=Origin(xyz=(0.163, 0.000, 0.035)),
        material=body_black,
        name="side_right",
    )
    body.visual(
        Box((0.312, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, -0.103, 0.027)),
        material=body_black,
        name="front_wall",
    )
    body.visual(
        Box((0.312, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, 0.103, 0.035)),
        material=body_black,
        name="rear_wall",
    )
    body.visual(
        Box((0.312, 0.130, 0.008)),
        origin=Origin(xyz=(0.0, -0.014, 0.056)),
        material=steel,
        name="deck",
    )
    body.visual(
        Box((0.182, 0.086, 0.050)),
        origin=Origin(xyz=(0.0, 0.074, 0.085)),
        material=body_black,
        name="head_block",
    )
    body.visual(
        Box((0.110, 0.040, 0.017)),
        origin=Origin(xyz=(0.0, 0.004, 0.0685)),
        material=dark_steel,
        name="die_block",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(-0.032, 0.004, 0.076), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="die_0",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.032, 0.004, 0.076), rpy=(0.0, 0.0, 0.0)),
        material=steel,
        name="die_1",
    )
    body.visual(
        Box((0.016, 0.024, 0.038)),
        origin=Origin(xyz=(-0.106, 0.100, 0.129)),
        material=dark_steel,
        name="cheek_0",
    )
    body.visual(
        Box((0.016, 0.024, 0.038)),
        origin=Origin(xyz=(0.106, 0.100, 0.129)),
        material=dark_steel,
        name="cheek_1",
    )
    body.visual(
        Box((0.196, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.100, 0.118)),
        material=dark_steel,
        name="hinge_bridge",
    )
    body.visual(
        Box((0.270, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.076, 0.063)),
        material=dark_steel,
        name="guide_track",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(-0.098, 0.082, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="door_hinge_0",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(xyz=(0.098, 0.082, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="door_hinge_1",
    )

    handle = model.part("handle")
    handle.inertial = Inertial.from_geometry(
        Box((0.21, 0.16, 0.09)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.070, -0.018)),
    )

    handle_loop = tube_from_spline_points(
        [
            (-0.072, 0.000, 0.000),
            (-0.074, -0.016, 0.004),
            (-0.079, -0.046, 0.010),
            (-0.074, -0.082, 0.002),
            (-0.052, -0.118, -0.010),
            (-0.020, -0.138, -0.017),
            (0.020, -0.138, -0.017),
            (0.052, -0.118, -0.010),
            (0.074, -0.082, 0.002),
            (0.079, -0.046, 0.010),
            (0.074, -0.016, 0.004),
            (0.072, 0.000, 0.000),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(
        _mesh("hole_punch_handle_loop", handle_loop),
        material=handle_coat,
        name="loop",
    )
    for sign, index in ((-1.0, 0), (1.0, 1)):
        handle.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(xyz=(sign * 0.079, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"hinge_collar_{index}",
        )
        handle.visual(
            Box((0.040, 0.018, 0.018)),
            origin=Origin(xyz=(sign * 0.079, -0.003, 0.004)),
            material=dark_steel,
            name=f"rear_link_{index}",
        )
        handle.visual(
            Box((0.016, 0.030, 0.040)),
            origin=Origin(xyz=(sign * 0.048, -0.114, -0.020)),
            material=dark_steel,
            name=f"front_link_{index}",
        )
        handle.visual(
            Cylinder(radius=0.008, length=0.026),
            origin=Origin(xyz=(sign * 0.032, -0.102, -0.050)),
            material=steel,
            name=f"punch_{index}",
        )
    handle.visual(
        Box((0.120, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.100, -0.029)),
        material=dark_steel,
        name="punch_beam",
    )

    guide = model.part("guide_bar")
    guide.inertial = Inertial.from_geometry(
        Box((0.24, 0.028, 0.028)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.002, 0.009)),
    )
    guide.visual(
        Box((0.230, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=guide_gray,
        name="bar",
    )
    guide.visual(
        Box((0.020, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.008, 0.013)),
        material=guide_gray,
        name="cursor",
    )
    guide.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(-0.082, 0.0, 0.002)),
        material=dark_steel,
        name="runner_0",
    )
    guide.visual(
        Box((0.018, 0.012, 0.004)),
        origin=Origin(xyz=(0.082, 0.0, 0.002)),
        material=dark_steel,
        name="runner_1",
    )

    door = model.part("waste_door")
    door.inertial = Inertial.from_geometry(
        Box((0.172, 0.060, 0.020)),
        mass=0.12,
        origin=Origin(xyz=(0.0, -0.028, -0.004)),
    )
    door.visual(
        Box((0.172, 0.052, 0.008)),
        origin=Origin(xyz=(0.0, -0.026, -0.004)),
        material=bin_black,
        name="panel",
    )
    door.visual(
        Box((0.040, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.052, -0.006)),
        material=dark_steel,
        name="pull",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, 0.108, 0.146)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.8, lower=0.0, upper=1.05),
    )
    model.articulation(
        "body_to_guide_bar",
        ArticulationType.PRISMATIC,
        parent=body,
        child=guide,
        origin=Origin(xyz=(0.0, -0.076, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.16, lower=-0.045, upper=0.045),
    )
    model.articulation(
        "body_to_waste_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, 0.082, 0.008)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.2, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    guide = object_model.get_part("guide_bar")
    door = object_model.get_part("waste_door")

    handle_joint = object_model.get_articulation("body_to_handle")
    guide_joint = object_model.get_articulation("body_to_guide_bar")
    door_joint = object_model.get_articulation("body_to_waste_door")

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="punch_0",
        negative_elem="die_0",
        min_gap=0.002,
        max_gap=0.008,
        name="left punch hovers just above die at rest",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="punch_1",
        negative_elem="die_1",
        min_gap=0.002,
        max_gap=0.008,
        name="right punch hovers just above die at rest",
    )
    ctx.expect_contact(
        guide,
        body,
        elem_a="runner_0",
        elem_b="deck",
        contact_tol=1e-6,
        name="guide runner stays seated on deck",
    )
    ctx.expect_overlap(
        guide,
        body,
        axes="x",
        elem_a="bar",
        elem_b="guide_track",
        min_overlap=0.180,
        name="guide bar remains captured by front track",
    )

    closed_loop = ctx.part_element_world_aabb(handle, elem="loop")
    guide_rest = ctx.part_world_position(guide)
    door_closed = ctx.part_element_world_aabb(door, elem="panel")

    with ctx.pose({handle_joint: 1.05, guide_joint: 0.045, door_joint: 1.20}):
        open_loop = ctx.part_element_world_aabb(handle, elem="loop")
        guide_shifted = ctx.part_world_position(guide)
        door_open = ctx.part_element_world_aabb(door, elem="panel")

    handle_lifts = (
        closed_loop is not None
        and open_loop is not None
        and open_loop[1][2] > closed_loop[1][2] + 0.055
    )
    ctx.check(
        "handle opens upward from the rear hinge",
        handle_lifts,
        details=f"closed={closed_loop}, open={open_loop}",
    )

    guide_slides = (
        guide_rest is not None
        and guide_shifted is not None
        and guide_shifted[0] > guide_rest[0] + 0.040
        and abs(guide_shifted[1] - guide_rest[1]) < 1e-6
        and abs(guide_shifted[2] - guide_rest[2]) < 1e-6
    )
    ctx.check(
        "guide bar slides laterally along the front edge",
        guide_slides,
        details=f"rest={guide_rest}, shifted={guide_shifted}",
    )

    door_swings_down = (
        door_closed is not None
        and door_open is not None
        and door_open[0][2] < door_closed[0][2] - 0.030
    )
    ctx.check(
        "waste door swings downward from the underside hinge",
        door_swings_down,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
