from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

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
    mesh_from_cadquery,
)


FOOT_X = 0.110
FOOT_Y = 0.090
FOOT_Z = 0.012

PEDESTAL_X = 0.080
PEDESTAL_Y = 0.070
PEDESTAL_Z = 0.168

HINGE_CENTER_Z = 0.102
HINGE_AXIS_X = 0.055

STRAP_X = 0.018
STRAP_Y = 0.024
STRAP_Z = 0.120

KNUCKLE_OUTER_R = 0.0115
PIN_RADIUS = 0.0072
PIN_BORE_R = 0.0082

FIXED_KNUCKLE_Z = 0.035
MOVING_KNUCKLE_Z = 0.037
KNUCKLE_GAP = 0.0015
BARREL_STACK_Z = 2.0 * FIXED_KNUCKLE_Z + MOVING_KNUCKLE_Z + 2.0 * KNUCKLE_GAP

DOOR_X = 0.082
DOOR_Y = 0.010
DOOR_Z = 0.114
DOOR_PANEL_OVERLAP = 0.0015
DOOR_PANEL_CENTER_X = KNUCKLE_OUTER_R + DOOR_X / 2.0 - DOOR_PANEL_OVERLAP


def _pedestal_body_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(FOOT_X, FOOT_Y, FOOT_Z).translate((0.0, 0.0, FOOT_Z / 2.0))
    body = cq.Workplane("XY").box(PEDESTAL_X, PEDESTAL_Y, PEDESTAL_Z).translate(
        (0.0, 0.0, FOOT_Z + PEDESTAL_Z / 2.0)
    )
    z_offset = MOVING_KNUCKLE_Z / 2.0 + KNUCKLE_GAP + FIXED_KNUCKLE_Z / 2.0
    back_plate = cq.Workplane("XY").box(0.003, STRAP_Y, STRAP_Z).translate(
        (PEDESTAL_X / 2.0 + 0.0015, 0.0, HINGE_CENTER_Z)
    )
    top_arm = cq.Workplane("XY").box(STRAP_X, STRAP_Y, FIXED_KNUCKLE_Z - 0.003).translate(
        (PEDESTAL_X / 2.0 + STRAP_X / 2.0, 0.0, HINGE_CENTER_Z + z_offset)
    )
    bottom_arm = cq.Workplane("XY").box(STRAP_X, STRAP_Y, FIXED_KNUCKLE_Z - 0.003).translate(
        (PEDESTAL_X / 2.0 + STRAP_X / 2.0, 0.0, HINGE_CENTER_Z - z_offset)
    )
    return foot.union(body).union(back_plate).union(top_arm).union(bottom_arm)


def _fixed_knuckles_shape() -> cq.Workplane:
    z_offset = MOVING_KNUCKLE_Z / 2.0 + KNUCKLE_GAP + FIXED_KNUCKLE_Z / 2.0

    top = cq.Workplane("XY").circle(KNUCKLE_OUTER_R).extrude(FIXED_KNUCKLE_Z).translate(
        (HINGE_AXIS_X, 0.0, HINGE_CENTER_Z + z_offset - FIXED_KNUCKLE_Z / 2.0)
    )
    bottom = cq.Workplane("XY").circle(KNUCKLE_OUTER_R).extrude(FIXED_KNUCKLE_Z).translate(
        (HINGE_AXIS_X, 0.0, HINGE_CENTER_Z - z_offset - FIXED_KNUCKLE_Z / 2.0)
    )
    return top.union(bottom)


def _door_panel_shape() -> cq.Workplane:
    panel = cq.Workplane("XY").box(DOOR_X, DOOR_Y, DOOR_Z).translate((DOOR_PANEL_CENTER_X, 0.0, 0.0))
    recess = cq.Workplane("XY").box(DOOR_X - 0.020, 0.003, DOOR_Z - 0.020).translate(
        (DOOR_PANEL_CENTER_X + 0.002, DOOR_Y / 2.0 - 0.0015, 0.0)
    )
    return panel.cut(recess)


def _moving_knuckle_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(KNUCKLE_OUTER_R).extrude(MOVING_KNUCKLE_Z).translate(
        (0.0, 0.0, -MOVING_KNUCKLE_Z / 2.0)
    )
    bore = cq.Workplane("XY").circle(PIN_BORE_R).extrude(MOVING_KNUCKLE_Z + 0.004).translate(
        (0.0, 0.0, -(MOVING_KNUCKLE_Z + 0.004) / 2.0)
    )
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_side_hinge")

    model.material("pedestal_coat", rgba=(0.26, 0.27, 0.29, 1.0))
    model.material("hinge_hardware", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("door_finish", rgba=(0.64, 0.67, 0.71, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(_pedestal_body_shape(), "pedestal_body"),
        material="pedestal_coat",
        name="pedestal_body",
    )
    pedestal.visual(
        mesh_from_cadquery(_fixed_knuckles_shape(), "fixed_knuckles"),
        material="hinge_hardware",
        name="fixed_knuckles",
    )
    pedestal.visual(
        Cylinder(radius=PIN_RADIUS, length=BARREL_STACK_Z),
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_CENTER_Z)),
        material="hinge_hardware",
        name="hinge_pin",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((FOOT_X, FOOT_Y, FOOT_Z + PEDESTAL_Z)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, (FOOT_Z + PEDESTAL_Z) / 2.0)),
    )

    door = model.part("door_plate")
    door.visual(
        mesh_from_cadquery(_door_panel_shape(), "door_panel"),
        material="door_finish",
        name="door_panel",
    )
    door.visual(
        mesh_from_cadquery(_moving_knuckle_shape(), "moving_knuckle"),
        material="hinge_hardware",
        name="moving_knuckle",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_X + KNUCKLE_OUTER_R + 0.010, DOOR_Y, DOOR_Z)),
        mass=1.4,
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_door",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=12.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    door = object_model.get_part("door_plate")
    hinge = object_model.get_articulation("pedestal_to_door")

    pedestal_body = pedestal.get_visual("pedestal_body")
    door_panel = door.get_visual("door_panel")

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            pedestal,
            axis="x",
            positive_elem=door_panel,
            negative_elem=pedestal_body,
            min_gap=0.004,
            max_gap=0.010,
            name="closed plate sits just outboard of the pedestal block",
        )
        ctx.expect_overlap(
            door,
            pedestal,
            axes="yz",
            elem_a=door_panel,
            elem_b=pedestal_body,
            min_overlap=0.008,
            name="closed plate lines up with the pedestal side face",
        )
        closed_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)

    with ctx.pose({hinge: 1.45}):
        open_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)

    closed_center = _center_from_aabb(closed_panel_aabb)
    open_center = _center_from_aabb(open_panel_aabb)
    ctx.check(
        "positive hinge rotation swings the plate outward toward +y",
        closed_center is not None
        and open_center is not None
        and open_center[1] > closed_center[1] + 0.040,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
