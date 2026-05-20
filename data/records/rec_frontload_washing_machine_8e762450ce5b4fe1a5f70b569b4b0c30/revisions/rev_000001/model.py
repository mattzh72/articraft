from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WASHER_WIDTH = 0.70
WASHER_DEPTH = 0.76
PEDESTAL_HEIGHT = 0.30
WASHER_HEIGHT = 0.90
FLOOR_CLEARANCE = 0.04
FRONT_Y = -WASHER_DEPTH / 2.0


def _cabinet_shell_mesh():
    """One connected white shell: washer body plus hollow pedestal frame."""

    body_center_z = FLOOR_CLEARANCE + PEDESTAL_HEIGHT + WASHER_HEIGHT / 2.0
    body = cq.Workplane("XY").box(WASHER_WIDTH, WASHER_DEPTH, WASHER_HEIGHT).translate(
        (0.0, 0.0, body_center_z)
    )
    try:
        body = body.edges("|Z").fillet(0.025)
    except Exception:
        pass

    pedestal_center_z = FLOOR_CLEARANCE + PEDESTAL_HEIGHT / 2.0
    pedestal = cq.Workplane("XY").box(
        WASHER_WIDTH, WASHER_DEPTH, PEDESTAL_HEIGHT
    ).translate((0.0, 0.0, pedestal_center_z))

    wall = 0.045
    back_wall = 0.045
    cavity_w = WASHER_WIDTH - 2.0 * wall
    cavity_h = PEDESTAL_HEIGHT - 2.0 * wall
    cavity_y0 = FRONT_Y - 0.006
    cavity_y1 = WASHER_DEPTH / 2.0 - back_wall
    cavity_d = cavity_y1 - cavity_y0
    cavity = cq.Workplane("XY").box(cavity_w, cavity_d, cavity_h).translate(
        (0.0, (cavity_y0 + cavity_y1) / 2.0, pedestal_center_z)
    )
    pedestal = pedestal.cut(cavity)

    return body.union(pedestal)


def _annular_disc_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness, both=True)
    )
    try:
        ring = ring.edges().fillet(min(0.006, thickness * 0.22))
    except Exception:
        pass
    return mesh_from_cadquery(ring, name, tolerance=0.001, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_drawer_washer_combo")

    white = model.material("warm_white_enamel", rgba=(0.93, 0.94, 0.92, 1.0))
    satin_white = model.material("satin_white_panel", rgba=(0.98, 0.98, 0.96, 1.0))
    light_gray = model.material("light_gray_plastic", rgba=(0.74, 0.76, 0.76, 1.0))
    dark = model.material("black_rubber_shadow", rgba=(0.015, 0.016, 0.017, 1.0))
    glass = model.material("smoked_blue_glass", rgba=(0.18, 0.28, 0.36, 0.48))
    chrome = model.material("brushed_chrome", rgba=(0.70, 0.72, 0.72, 1.0))
    drum = model.material("stainless_drum", rgba=(0.55, 0.56, 0.54, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(
            _cabinet_shell_mesh(),
            "cabinet_shell",
            tolerance=0.0015,
            angular_tolerance=0.08,
        ),
        material=white,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.63, 0.014, 0.105)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.007, 1.165)),
        material=light_gray,
        name="control_strip",
    )
    cabinet.visual(
        _annular_disc_mesh(0.235, 0.158, 0.018, "door_gasket"),
        origin=Origin(xyz=(-0.020, FRONT_Y - 0.009, 0.805), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="door_gasket",
    )
    cabinet.visual(
        Cylinder(radius=0.160, length=0.014),
        origin=Origin(xyz=(-0.020, FRONT_Y - 0.015, 0.805), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=drum,
        name="drum_face",
    )
    cabinet.visual(
        Cylinder(radius=0.110, length=0.004),
        origin=Origin(xyz=(-0.020, FRONT_Y - 0.018, 0.805), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="drum_shadow",
    )
    cabinet.visual(
        Box((0.61, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.006, FLOOR_CLEARANCE + PEDESTAL_HEIGHT + 0.012)),
        material=light_gray,
        name="washer_pedestal_seam",
    )
    cabinet.visual(
        Box((0.028, 0.580, 0.026)),
        origin=Origin(xyz=(-0.291, -0.030, 0.190)),
        material=chrome,
        name="outer_rail_0",
    )
    cabinet.visual(
        Box((0.028, 0.580, 0.026)),
        origin=Origin(xyz=(0.291, -0.030, 0.190)),
        material=chrome,
        name="outer_rail_1",
    )
    for i, z in enumerate((0.635, 0.975)):
        cabinet.visual(
            Box((0.060, 0.060, 0.090)),
            origin=Origin(xyz=(-0.305, FRONT_Y - 0.028, z)),
            material=chrome,
            name=f"cabinet_hinge_leaf_{i}",
        )

    # Door child frame sits on the vertical hinge pin.  The round door extends in +X.
    door = model.part("door")
    door.visual(
        _annular_disc_mesh(0.246, 0.162, 0.046, "door_outer_ring"),
        origin=Origin(xyz=(0.285, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="outer_ring",
    )
    door.visual(
        Cylinder(radius=0.173, length=0.018),
        origin=Origin(xyz=(0.285, -0.006, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="glass_bowl",
    )
    door.visual(
        Box((0.038, 0.026, 0.145)),
        origin=Origin(xyz=(0.505, -0.033, 0.0)),
        material=light_gray,
        name="door_pull",
    )
    for i, z in enumerate((-0.170, 0.170)):
        door.visual(
            Cylinder(radius=0.022, length=0.132),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=chrome,
            name=f"hinge_barrel_{i}",
        )
        door.visual(
            Box((0.092, 0.014, 0.090)),
            origin=Origin(xyz=(0.045, -0.004, z)),
            material=chrome,
            name=f"hinge_leaf_{i}",
        )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.305, FRONT_Y - 0.080, 0.805)),
        # Negative Z makes positive q swing the right/free side outward toward -Y.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.85),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.640, 0.050, 0.240)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_white,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.026, 0.560, 0.160)),
        origin=Origin(xyz=(-0.232, 0.305, -0.010)),
        material=satin_white,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((0.026, 0.560, 0.160)),
        origin=Origin(xyz=(0.232, 0.305, -0.010)),
        material=satin_white,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((0.464, 0.560, 0.018)),
        origin=Origin(xyz=(0.0, 0.305, -0.088)),
        material=light_gray,
        name="drawer_floor",
    )
    drawer.visual(
        Box((0.464, 0.026, 0.160)),
        origin=Origin(xyz=(0.0, 0.585, -0.010)),
        material=satin_white,
        name="drawer_back",
    )
    drawer.visual(
        Box((0.016, 0.580, 0.022)),
        origin=Origin(xyz=(-0.253, 0.310, 0.0)),
        material=chrome,
        name="drawer_rail_0",
    )
    drawer.visual(
        Box((0.016, 0.580, 0.022)),
        origin=Origin(xyz=(0.253, 0.310, 0.0)),
        material=chrome,
        name="drawer_rail_1",
    )
    drawer.visual(
        Cylinder(radius=0.014, length=0.440),
        origin=Origin(xyz=(0.0, -0.070, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="bar_handle",
    )
    for x in (-0.185, 0.185):
        drawer.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(xyz=(x, -0.043, 0.030), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"handle_post_{0 if x < 0 else 1}",
        )

    model.articulation(
        "pedestal_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.030, 0.190)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.460),
    )

    for side, x in enumerate((-0.269, 0.269)):
        rail = model.part(f"rail_{side}")
        rail.visual(
            Box((0.016, 0.540, 0.020)),
            origin=Origin(xyz=(0.0, 0.270, 0.0)),
            material=chrome,
            name="rail_stage",
        )
        model.articulation(
            f"cabinet_to_rail_{side}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=rail,
            origin=Origin(xyz=(x, -0.300, 0.190)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=40.0, velocity=0.25, lower=0.0, upper=0.230),
            mimic=Mimic(joint="pedestal_to_drawer", multiplier=0.5, offset=0.0),
        )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.037, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=satin_white,
        name="knob_cap",
    )
    knob.visual(
        Box((0.006, 0.036, 0.004)),
        origin=Origin(xyz=(0.0, 0.012, 0.033)),
        material=dark,
        name="knob_indicator",
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(-0.205, FRONT_Y - 0.014, 1.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-2.6, upper=2.6),
    )

    for i, x in enumerate((0.095, 0.150, 0.205)):
        button = model.part(f"button_{i}")
        button.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=satin_white,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.014, 1.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=-0.004, upper=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")
    door_joint = object_model.get_articulation("cabinet_to_door")
    drawer_joint = object_model.get_articulation("pedestal_to_drawer")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.006,
        max_gap=0.060,
        positive_elem="door_gasket",
        negative_elem="outer_ring",
        name="closed washer door seats just ahead of gasket",
    )
    ctx.expect_gap(
        cabinet,
        drawer,
        axis="y",
        min_gap=0.002,
        max_gap=0.018,
        positive_elem="cabinet_shell",
        negative_elem="drawer_front",
        name="drawer front is proud of pedestal face",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        min_overlap=0.30,
        elem_a="drawer_rail_0",
        elem_b="outer_rail_0",
        name="closed drawer rail is deeply engaged",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        min_overlap=0.30,
        elem_a="drawer_rail_1",
        elem_b="outer_rail_1",
        name="opposite closed drawer rail is deeply engaged",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward on left hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_drawer_aabb = ctx.part_world_aabb(drawer)
    with ctx.pose({drawer_joint: 0.460}):
        open_drawer_aabb = ctx.part_world_aabb(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            min_overlap=0.035,
            elem_a="drawer_rail_0",
            elem_b="outer_rail_0",
            name="extended rail still has retained insertion",
        )
    ctx.check(
        "pedestal drawer reaches full extension",
        closed_drawer_aabb is not None
        and open_drawer_aabb is not None
        and open_drawer_aabb[0][1] < closed_drawer_aabb[0][1] - 0.40,
        details=f"closed={closed_drawer_aabb}, open={open_drawer_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
