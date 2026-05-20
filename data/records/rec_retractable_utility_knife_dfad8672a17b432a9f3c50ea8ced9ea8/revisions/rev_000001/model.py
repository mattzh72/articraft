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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_utility_knife")

    handle_aluminum = model.material("handle_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.18, 0.18, 0.20, 1.0))
    slider_black = model.material("slider_black", rgba=(0.10, 0.10, 0.12, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.84, 0.86, 0.89, 1.0))
    latch_orange = model.material("latch_orange", rgba=(0.89, 0.42, 0.10, 1.0))

    handle_length = 0.165
    handle_half_width = 0.0075
    handle_wall = 0.002
    inner_width = handle_half_width * 2.0 - 2.0 * handle_wall

    handle_side_profile = [
        (-0.079, 0.000),
        (-0.076, 0.008),
        (-0.066, 0.0113),
        (-0.032, 0.0130),
        (0.028, 0.0130),
        (0.056, 0.0116),
        (0.071, 0.0090),
        (0.080, 0.0042),
        (0.081, -0.0012),
        (0.074, -0.0070),
        (0.058, -0.0110),
        (0.018, -0.0130),
        (-0.038, -0.0130),
        (-0.070, -0.0100),
        (-0.081, -0.0040),
    ]
    handle_side_geom = ExtrudeGeometry(handle_side_profile, handle_wall, center=True).rotate_x(
        math.pi / 2.0
    )
    handle_side_mesh = mesh_from_geometry(handle_side_geom, "utility_knife_handle_side")

    blade_profile = [
        (0.000, -0.0028),
        (0.021, -0.0026),
        (0.029, -0.0010),
        (0.035, 0.0030),
        (0.027, 0.0042),
        (0.004, 0.0038),
    ]
    blade_geom = ExtrudeGeometry(blade_profile, 0.0012, center=True).rotate_x(math.pi / 2.0)
    blade_mesh = mesh_from_geometry(blade_geom, "utility_knife_blade")

    handle = model.part("handle_shell")
    handle.visual(
        handle_side_mesh,
        origin=Origin(xyz=(0.0, -(handle_half_width - handle_wall / 2.0), 0.0)),
        material=handle_aluminum,
        name="left_shell",
    )
    handle.visual(
        handle_side_mesh,
        origin=Origin(xyz=(0.0, handle_half_width - handle_wall / 2.0, 0.0)),
        material=handle_aluminum,
        name="right_shell",
    )
    handle.visual(
        Box((0.138, inner_width, 0.003)),
        origin=Origin(xyz=(-0.004, 0.0, -0.0115)),
        material=handle_aluminum,
        name="track_floor",
    )
    handle.visual(
        Box((0.006, handle_half_width * 2.0, 0.018)),
        origin=Origin(xyz=(-0.079, 0.0, -0.0005)),
        material=handle_aluminum,
        name="tail_cap",
    )
    handle.visual(
        Box((0.035, handle_half_width * 2.0, 0.002)),
        origin=Origin(xyz=(-0.053, 0.0, 0.0107)),
        material=handle_aluminum,
        name="rear_top_bridge",
    )
    handle.visual(
        Box((0.022, handle_half_width * 2.0, 0.002)),
        origin=Origin(xyz=(0.061, 0.0, 0.0112)),
        material=handle_aluminum,
        name="front_top_bridge",
    )
    handle.visual(
        Box((0.020, handle_half_width * 2.0, 0.003)),
        origin=Origin(xyz=(0.067, 0.0, -0.0105)),
        material=handle_aluminum,
        name="nose_keel",
    )
    handle.visual(
        Box((0.070, 0.0012, 0.010)),
        origin=Origin(xyz=(-0.010, -0.0081, -0.002)),
        material=grip_rubber,
        name="left_grip_inset",
    )
    handle.visual(
        Box((0.045, 0.0012, 0.010)),
        origin=Origin(xyz=(0.026, 0.0081, -0.002)),
        material=grip_rubber,
        name="right_front_grip_inset",
    )
    handle.visual(
        Box((0.026, 0.0012, 0.012)),
        origin=Origin(xyz=(-0.006, 0.0081, -0.001)),
        material=grip_rubber,
        name="latch_mount_pad",
    )
    handle.visual(
        Cylinder(radius=0.0038, length=0.0016),
        origin=Origin(xyz=(-0.010, 0.0083, -0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=handle_aluminum,
        name="handle_latch_boss",
    )
    handle.inertial = Inertial.from_geometry(
        Box((handle_length, handle_half_width * 2.0, 0.026)),
        mass=0.19,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    carriage = model.part("blade_carriage")
    carriage.visual(
        Box((0.050, 0.0110, 0.006)),
        origin=Origin(xyz=(0.025, 0.0, -0.002)),
        material=slider_black,
        name="carriage_runner",
    )
    carriage.visual(
        Box((0.020, 0.0075, 0.005)),
        origin=Origin(xyz=(0.039, 0.0, -0.0015)),
        material=slider_black,
        name="blade_clamp",
    )
    carriage.visual(
        Box((0.010, 0.0030, 0.017)),
        origin=Origin(xyz=(0.022, 0.0, 0.0065)),
        material=slider_black,
        name="thumb_stem",
    )
    carriage.visual(
        Box((0.018, 0.0095, 0.004)),
        origin=Origin(xyz=(0.022, 0.0, 0.015)),
        material=slider_black,
        name="thumb_pad",
    )
    for index, ridge_x in enumerate((-0.004, 0.0, 0.004)):
        carriage.visual(
            Box((0.0015, 0.0080, 0.0016)),
            origin=Origin(xyz=(0.022 + ridge_x, 0.0, 0.0172)),
            material=slider_black,
            name=f"thumb_ridge_{index}",
        )
    carriage.visual(
        blade_mesh,
        origin=Origin(xyz=(0.050, 0.0, -0.0008)),
        material=blade_steel,
        name="blade_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.086, 0.010, 0.021)),
        mass=0.05,
        origin=Origin(xyz=(0.043, 0.0, 0.004)),
    )

    latch = model.part("safety_latch")
    latch.visual(
        Cylinder(radius=0.0046, length=0.0018),
        origin=Origin(xyz=(0.0, 0.0009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_orange,
        name="latch_pivot",
    )
    latch.visual(
        Box((0.018, 0.0018, 0.008)),
        origin=Origin(xyz=(0.009, 0.0009, -0.001)),
        material=latch_orange,
        name="latch_tab",
    )
    latch.visual(
        Box((0.006, 0.0018, 0.010)),
        origin=Origin(xyz=(0.0175, 0.0009, -0.001)),
        material=latch_orange,
        name="latch_tip",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.022, 0.002, 0.012)),
        mass=0.01,
        origin=Origin(xyz=(0.010, 0.0009, -0.001)),
    )

    model.articulation(
        "handle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(xyz=(-0.029, 0.0, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.20,
            lower=0.0,
            upper=0.040,
        ),
    )

    model.articulation(
        "handle_to_latch",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=latch,
        origin=Origin(xyz=(-0.010, 0.0091, -0.001)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.0,
            lower=0.0,
            upper=0.55,
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

    handle = object_model.get_part("handle_shell")
    carriage = object_model.get_part("blade_carriage")
    latch = object_model.get_part("safety_latch")
    carriage_slide = object_model.get_articulation("handle_to_carriage")
    latch_hinge = object_model.get_articulation("handle_to_latch")

    carriage_upper = 0.040
    latch_upper = 0.55

    with ctx.pose({carriage_slide: 0.0}):
        ctx.expect_within(
            carriage,
            handle,
            axes="yz",
            inner_elem="carriage_runner",
            margin=0.0,
            name="carriage runner stays centered in the handle track at rest",
        )
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_runner",
            min_overlap=0.045,
            name="carriage runner remains well inserted at rest",
        )
        ctx.expect_contact(
            latch,
            handle,
            elem_a="latch_pivot",
            elem_b="handle_latch_boss",
            name="safety latch sits on the handle pivot boss",
        )
        rest_carriage_pos = ctx.part_world_position(carriage)
        rest_blade_aabb = ctx.part_element_world_aabb(carriage, elem="blade_plate")

    with ctx.pose({carriage_slide: carriage_upper}):
        ctx.expect_within(
            carriage,
            handle,
            axes="yz",
            inner_elem="carriage_runner",
            margin=0.0,
            name="carriage runner stays centered in the handle track when extended",
        )
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_runner",
            min_overlap=0.020,
            name="carriage runner retains hidden insertion when extended",
        )
        extended_carriage_pos = ctx.part_world_position(carriage)
        extended_blade_aabb = ctx.part_element_world_aabb(carriage, elem="blade_plate")

    ctx.check(
        "blade carriage slides toward the knife nose",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.030
        and rest_blade_aabb is not None
        and extended_blade_aabb is not None
        and extended_blade_aabb[1][0] > rest_blade_aabb[1][0] + 0.030,
        details=(
            f"rest_pos={rest_carriage_pos}, extended_pos={extended_carriage_pos}, "
            f"rest_blade={rest_blade_aabb}, extended_blade={extended_blade_aabb}"
        ),
    )

    with ctx.pose({latch_hinge: 0.0}):
        closed_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_tab")

    with ctx.pose({latch_hinge: latch_upper}):
        open_latch_aabb = ctx.part_element_world_aabb(latch, elem="latch_tab")
        ctx.expect_contact(
            latch,
            handle,
            elem_a="latch_pivot",
            elem_b="handle_latch_boss",
            name="safety latch remains seated on its boss while rotated",
        )

    ctx.check(
        "safety latch rotates upward in the handle side plane",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[1][2] > closed_latch_aabb[1][2] + 0.004,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
