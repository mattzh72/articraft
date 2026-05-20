from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

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
    model = ArticulatedObject(name="wheelie_bin")

    body_green = model.material("body_green", rgba=(0.19, 0.39, 0.20, 1.0))
    lid_green = model.material("lid_green", rgba=(0.17, 0.36, 0.18, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.34, 0.36, 0.38, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.17, 0.18, 0.19, 1.0))

    wall_t = 0.007
    floor_t = 0.014
    body_height = 0.83
    bottom_z = 0.12
    body_mid_z = bottom_z + body_height * 0.5
    top_z = bottom_z + body_height

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((0.62, 0.74, 1.02)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    body.visual(
        Box((0.49, 0.60, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_z + floor_t * 0.5)),
        material=body_green,
        name="floor",
    )
    body.visual(
        Box((0.54, wall_t, 0.785)),
        origin=Origin(xyz=(0.0, 0.316, bottom_z + 0.785 * 0.5), rpy=(-0.10, 0.0, 0.0)),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((0.49, wall_t, body_height)),
        origin=Origin(xyz=(0.0, -0.308, body_mid_z)),
        material=body_green,
        name="rear_wall",
    )
    body.visual(
        Box((wall_t, 0.64, body_height)),
        origin=Origin(xyz=(0.252, 0.006, body_mid_z), rpy=(0.0, -0.09, 0.0)),
        material=body_green,
        name="left_wall",
    )
    body.visual(
        Box((wall_t, 0.64, body_height)),
        origin=Origin(xyz=(-0.252, 0.006, body_mid_z), rpy=(0.0, 0.09, 0.0)),
        material=body_green,
        name="right_wall",
    )

    body.visual(
        Box((0.55, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.339, top_z - 0.014)),
        material=body_green,
        name="front_rim",
    )
    body.visual(
        Box((0.028, 0.65, 0.028)),
        origin=Origin(xyz=(0.275, 0.012, top_z - 0.014)),
        material=body_green,
        name="left_rim",
    )
    body.visual(
        Box((0.028, 0.65, 0.028)),
        origin=Origin(xyz=(-0.275, 0.012, top_z - 0.014)),
        material=body_green,
        name="right_rim",
    )
    body.visual(
        Box((0.23, 0.045, 0.036)),
        origin=Origin(xyz=(0.0, -0.286, top_z - 0.020)),
        material=body_green,
        name="rear_handle_bridge",
    )
    body.visual(
        Box((0.040, 0.060, 0.140)),
        origin=Origin(xyz=(0.230, -0.286, 0.180)),
        material=body_green,
        name="left_axle_pocket",
    )
    body.visual(
        Box((0.040, 0.060, 0.140)),
        origin=Origin(xyz=(-0.230, -0.286, 0.180)),
        material=body_green,
        name="right_axle_pocket",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.56),
        origin=Origin(xyz=(0.0, -0.300, 0.110), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_steel,
        name="axle_tube",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.280, -0.300, 0.110), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_steel,
        name="left_axle_stub",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(-0.280, -0.300, 0.110), rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_steel,
        name="right_axle_stub",
    )

    hinge_y = -0.322
    hinge_z = top_z + 0.012
    for side_name, x_pos in (("left", 0.095), ("right", -0.095)):
        body.visual(
            Cylinder(radius=0.012, length=0.110),
            origin=Origin(xyz=(x_pos, hinge_y, hinge_z), rpy=(0.0, pi / 2.0, 0.0)),
            material=axle_steel,
            name=f"{side_name}_hinge_barrel",
        )
        body.visual(
            Box((0.028, 0.040, 0.034)),
            origin=Origin(xyz=(x_pos, hinge_y + 0.016, top_z + 0.002)),
            material=body_green,
            name=f"{side_name}_hinge_bracket",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.60, 0.72, 0.10)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.34, 0.02)),
    )
    lid.visual(
        Box((0.60, 0.71, 0.012)),
        origin=Origin(xyz=(0.0, 0.355, 0.020), rpy=(-0.045, 0.0, 0.0)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.022, 0.67, 0.020)),
        origin=Origin(xyz=(0.288, 0.345, 0.014)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.022, 0.67, 0.020)),
        origin=Origin(xyz=(-0.288, 0.345, 0.014)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.56, 0.032, 0.032)),
        origin=Origin(xyz=(0.0, 0.700, 0.014)),
        material=lid_green,
        name="front_skirt",
    )
    lid.visual(
        Box((0.18, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.716, -0.010)),
        material=lid_green,
        name="front_grip",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.080),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_steel,
        name="lid_hinge_barrel",
    )
    lid.visual(
        Box((0.070, 0.024, 0.032)),
        origin=Origin(xyz=(0.0, 0.024, 0.016)),
        material=lid_green,
        name="lid_hinge_web",
    )
    lid.visual(
        Box((0.12, 0.028, 0.028)),
        origin=Origin(xyz=(0.0, 0.050, 0.0)),
        material=lid_green,
        name="lid_rear_stop",
    )

    def add_wheel(part_name: str, outer_cap_sign: float):
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.108, length=0.044),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.067, length=0.050),
            origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
            material=hub_grey,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.030, length=0.026),
            origin=Origin(
                xyz=(-outer_cap_sign * 0.012, 0.0, 0.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hub_grey,
            name="inner_boss",
        )
        wheel.visual(
            Box((0.008, 0.020, 0.054)),
            origin=Origin(xyz=(outer_cap_sign * 0.025, 0.0, 0.020)),
            material=hub_grey,
            name="hub_rib",
        )
        return wheel

    left_wheel = add_wheel("left_wheel", 1.0)
    right_wheel = add_wheel("right_wheel", -1.0)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(110.0),
        ),
    )
    wheel_center_y = -0.300
    wheel_center_z = 0.110
    model.articulation(
        "body_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=left_wheel,
        origin=Origin(xyz=(0.330, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
    )
    model.articulation(
        "body_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=right_wheel,
        origin=Origin(xyz=(-0.330, wheel_center_y, wheel_center_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=18.0),
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

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid_hinge = object_model.get_articulation("body_to_lid")
    left_wheel_joint = object_model.get_articulation("body_to_left_wheel")
    right_wheel_joint = object_model.get_articulation("body_to_right_wheel")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_rim",
            min_gap=0.0,
            max_gap=0.040,
            name="closed lid sits just above the front rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="lid_panel",
            elem_b="front_rim",
            min_overlap=0.50,
            name="lid spans the full bin opening width",
        )
        ctx.expect_contact(
            left_wheel,
            body,
            elem_a="inner_boss",
            elem_b="left_axle_stub",
            name="left wheel rides on the axle stub",
        )
        ctx.expect_contact(
            right_wheel,
            body,
            elem_a="inner_boss",
            elem_b="right_axle_stub",
            name="right wheel rides on the axle stub",
        )

    with ctx.pose({lid_hinge: radians(75.0)}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_grip")
    closed_front = ctx.part_element_world_aabb(lid, elem="front_grip")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_front is not None
        and open_front is not None
        and open_front[1][2] > closed_front[1][2] + 0.22
        and open_front[0][1] < closed_front[0][1] - 0.20,
        details=(
            f"closed_front={closed_front}, open_front={open_front}"
        ),
    )

    left_pos = ctx.part_world_position(left_wheel)
    right_pos = ctx.part_world_position(right_wheel)
    ctx.check(
        "rear wheels sit symmetrically on the axle line",
        left_pos is not None
        and right_pos is not None
        and abs(left_pos[0] + right_pos[0]) <= 0.002
        and abs(left_pos[1] - right_pos[1]) <= 1e-6
        and abs(left_pos[2] - right_pos[2]) <= 1e-6
        and left_pos[0] > 0.30
        and right_pos[0] < -0.30,
        details=f"left_pos={left_pos}, right_pos={right_pos}",
    )

    left_rib_rest = ctx.part_element_world_aabb(left_wheel, elem="hub_rib")
    with ctx.pose({left_wheel_joint: pi / 2.0}):
        left_rib_turned = ctx.part_element_world_aabb(left_wheel, elem="hub_rib")
    ctx.check(
        "left wheel spins about the horizontal axle",
        left_pos is not None
        and left_rib_rest is not None
        and left_rib_turned is not None
        and ((left_rib_rest[0][2] + left_rib_rest[1][2]) * 0.5) > left_pos[2] + 0.012
        and ((left_rib_turned[0][1] + left_rib_turned[1][1]) * 0.5) < left_pos[1] - 0.012,
        details=(
            f"left_pos={left_pos}, left_rib_rest={left_rib_rest}, "
            f"left_rib_turned={left_rib_turned}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
