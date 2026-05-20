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
    model = ArticulatedObject(name="wall_mounted_mailbox")

    wall_metal = model.material("wall_metal", rgba=(0.24, 0.27, 0.30, 1.0))
    cap_metal = model.material("cap_metal", rgba=(0.16, 0.18, 0.20, 1.0))
    door_paint = model.material("door_paint", rgba=(0.30, 0.33, 0.36, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.60, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.010, 0.352, 0.500)),
        origin=Origin(xyz=(-0.005, 0.0, 0.250)),
        material=wall_metal,
        name="mount_plate",
    )
    body.visual(
        Box((0.008, 0.320, 0.460)),
        origin=Origin(xyz=(0.004, 0.0, 0.230)),
        material=wall_metal,
        name="back_panel",
    )
    body.visual(
        Box((0.152, 0.010, 0.460)),
        origin=Origin(xyz=(0.084, -0.155, 0.230)),
        material=wall_metal,
        name="left_wall",
    )
    body.visual(
        Box((0.152, 0.010, 0.460)),
        origin=Origin(xyz=(0.084, 0.155, 0.230)),
        material=wall_metal,
        name="right_wall",
    )
    body.visual(
        Box((0.152, 0.300, 0.010)),
        origin=Origin(xyz=(0.084, 0.0, 0.005)),
        material=wall_metal,
        name="bottom_panel",
    )
    body.visual(
        Box((0.152, 0.320, 0.010)),
        origin=Origin(xyz=(0.084, 0.0, 0.455)),
        material=wall_metal,
        name="top_panel",
    )
    body.visual(
        Box((0.010, 0.018, 0.430)),
        origin=Origin(xyz=(0.155, -0.151, 0.225)),
        material=wall_metal,
        name="left_rim",
    )
    body.visual(
        Box((0.010, 0.018, 0.430)),
        origin=Origin(xyz=(0.155, 0.151, 0.225)),
        material=wall_metal,
        name="right_rim",
    )
    body.visual(
        Box((0.010, 0.284, 0.018)),
        origin=Origin(xyz=(0.155, 0.0, 0.441)),
        material=wall_metal,
        name="top_rim",
    )
    body.visual(
        Box((0.010, 0.284, 0.020)),
        origin=Origin(xyz=(0.155, 0.0, 0.010)),
        material=wall_metal,
        name="bottom_rim",
    )
    body.visual(
        Box((0.010, 0.010, 0.140)),
        origin=Origin(xyz=(0.157, -0.165, 0.230)),
        material=cap_metal,
        name="body_hinge_bracket",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.164, -0.170, 0.230)),
        material=cap_metal,
        name="body_hinge_barrel",
    )
    body.visual(
        Box((0.205, 0.352, 0.012)),
        origin=Origin(xyz=(0.090, 0.0, 0.470), rpy=(0.0, -0.10, 0.0)),
        material=cap_metal,
        name="rain_cap",
    )
    body.visual(
        Box((0.016, 0.338, 0.018)),
        origin=Origin(xyz=(0.186, 0.0, 0.468)),
        material=cap_metal,
        name="drip_lip",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.006, -0.090, 0.398), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="mount_bolt_upper_left",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.006, 0.090, 0.398), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="mount_bolt_upper_right",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.006, -0.090, 0.102), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="mount_bolt_lower_left",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(-0.006, 0.090, 0.102), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="mount_bolt_lower_right",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.220, 0.360, 0.500)),
        mass=6.5,
        origin=Origin(xyz=(0.080, 0.0, 0.250)),
    )

    door = model.part("front_door")
    door.visual(
        Box((0.012, 0.318, 0.444)),
        origin=Origin(xyz=(0.002, 0.171, 0.0)),
        material=door_paint,
        name="door_panel",
    )
    door.visual(
        Box((0.004, 0.274, 0.030)),
        origin=Origin(xyz=(0.010, 0.171, 0.177)),
        material=cap_metal,
        name="door_top_rail",
    )
    door.visual(
        Box((0.004, 0.274, 0.030)),
        origin=Origin(xyz=(0.010, 0.171, -0.177)),
        material=cap_metal,
        name="door_bottom_rail",
    )
    door.visual(
        Box((0.004, 0.032, 0.370)),
        origin=Origin(xyz=(0.010, 0.016, 0.0)),
        material=cap_metal,
        name="door_hinge_stile",
    )
    door.visual(
        Box((0.004, 0.026, 0.370)),
        origin=Origin(xyz=(0.010, 0.304, 0.0)),
        material=cap_metal,
        name="door_latch_stile",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=cap_metal,
        name="door_hinge_barrel_upper",
    )
    door.visual(
        Cylinder(radius=0.008, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, -0.176)),
        material=cap_metal,
        name="door_hinge_barrel_lower",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.016, 0.318, 0.444)),
        mass=1.8,
        origin=Origin(xyz=(0.002, 0.171, 0.0)),
    )

    model.articulation(
        "body_to_front_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.164, -0.170, 0.225)),
        # The closed door extends along local +Y from the hinge line.
        # -Z makes positive motion swing the free edge outward toward +X.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=1.95,
        ),
    )

    latch = model.part("rotary_latch")
    latch.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="backplate",
    )
    latch.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="pivot_boss",
    )
    latch.visual(
        Box((0.005, 0.012, 0.050)),
        origin=Origin(xyz=(0.0165, 0.0, 0.0)),
        material=brass,
        name="thumb_tab",
    )
    latch.visual(
        Cylinder(radius=0.004, length=0.006),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_metal,
        name="tab_end_cap",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.024, 0.040, 0.050)),
        mass=0.12,
        origin=Origin(xyz=(0.012, 0.0, 0.0)),
    )

    model.articulation(
        "door_to_rotary_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(0.008, 0.302, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("front_door")
    latch = object_model.get_part("rotary_latch")
    door_hinge = object_model.get_articulation("body_to_front_door")
    latch_joint = object_model.get_articulation("door_to_rotary_latch")

    ctx.check("body part exists", body is not None)
    ctx.check("door part exists", door is not None)
    ctx.check("latch part exists", latch is not None)

    with ctx.pose({door_hinge: 0.0, latch_joint: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="x",
            positive_elem="door_panel",
            negative_elem="top_panel",
            max_gap=0.012,
            max_penetration=0.0,
            name="door sits flush against the front opening plane",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            elem_a="door_panel",
            min_overlap=0.250,
            name="door covers the mailbox opening footprint",
        )
        ctx.expect_contact(
            latch,
            door,
            elem_a="backplate",
            elem_b="door_panel",
            name="rotary latch is mounted onto the door face",
        )

        closed_latch_pos = ctx.part_world_position(latch)
        closed_tab_aabb = ctx.part_element_world_aabb(latch, elem="thumb_tab")

    with ctx.pose({door_hinge: 1.20}):
        open_latch_pos = ctx.part_world_position(latch)

    ctx.check(
        "door opens outward on its vertical hinge axis",
        closed_latch_pos is not None
        and open_latch_pos is not None
        and open_latch_pos[0] > closed_latch_pos[0] + 0.12,
        details=f"closed={closed_latch_pos}, open={open_latch_pos}",
    )

    with ctx.pose({latch_joint: 1.10}):
        turned_tab_aabb = ctx.part_element_world_aabb(latch, elem="thumb_tab")

    def _extent(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis_index] - aabb[0][axis_index]

    closed_tab_y = _extent(closed_tab_aabb, 1)
    closed_tab_z = _extent(closed_tab_aabb, 2)
    turned_tab_y = _extent(turned_tab_aabb, 1)
    turned_tab_z = _extent(turned_tab_aabb, 2)
    ctx.check(
        "latch thumb tab rotates about its short front-back pivot",
        closed_tab_y is not None
        and closed_tab_z is not None
        and turned_tab_y is not None
        and turned_tab_z is not None
        and closed_tab_z > closed_tab_y * 2.0
        and turned_tab_y > turned_tab_z * 1.2,
        details=(
            f"closed_y={closed_tab_y}, closed_z={closed_tab_z}, "
            f"turned_y={turned_tab_y}, turned_z={turned_tab_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
