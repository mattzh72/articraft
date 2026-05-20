from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> object:
    """CadQuery rounded cabinet body, centered on its local origin."""
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_microwave")

    enamel = model.material("warm_cream_enamel", rgba=(0.86, 0.75, 0.55, 1.0))
    darker_enamel = model.material("slightly_darker_side_panels", rgba=(0.67, 0.55, 0.38, 1.0))
    black = model.material("black_bakelite", rgba=(0.025, 0.023, 0.020, 1.0))
    dark_glass = model.material("smoky_blue_glass", rgba=(0.10, 0.20, 0.27, 0.45))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.79, 0.72, 1.0))
    brass = model.material("aged_brass_hinge", rgba=(0.63, 0.45, 0.21, 1.0))
    red = model.material("red_badge", rgba=(0.72, 0.06, 0.035, 1.0))

    # Cabinet coordinate frame: X is width, negative Y is the front, Z is up.
    body_w, body_d, body_h = 0.66, 0.42, 0.35
    body_center_z = 0.18
    body_front_y = -body_d / 2.0
    panel_front_y = body_front_y - 0.0225

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_rounded_box((body_w, body_d, body_h), 0.035), "rounded_cabinet"),
        origin=Origin(xyz=(0.0, 0.0, body_center_z)),
        material=enamel,
        name="rounded_cabinet",
    )
    cabinet.visual(
        Box((0.150, 0.025, 0.300)),
        origin=Origin(xyz=(0.225, body_front_y - 0.010, body_center_z)),
        material=darker_enamel,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.340, 0.006, 0.205)),
        origin=Origin(xyz=(-0.105, body_front_y - 0.003, body_center_z)),
        material=black,
        name="oven_cavity_shadow",
    )
    cabinet.visual(
        Box((0.485, 0.010, 0.016)),
        origin=Origin(xyz=(-0.085, body_front_y - 0.005, 0.340)),
        material=chrome,
        name="front_brow_trim",
    )
    cabinet.visual(
        Box((0.485, 0.010, 0.012)),
        origin=Origin(xyz=(-0.085, body_front_y - 0.005, 0.020)),
        material=chrome,
        name="front_sill_trim",
    )
    cabinet.visual(
        Box((0.060, 0.006, 0.020)),
        origin=Origin(xyz=(0.225, panel_front_y - 0.001, 0.318)),
        material=red,
        name="brand_badge",
    )

    for i, x in enumerate((-0.230, 0.230)):
        cabinet.visual(
            Cylinder(radius=0.030, length=0.024),
            origin=Origin(xyz=(x, -0.120, -0.007)),
            material=black,
            name=f"front_foot_{i}",
        )
        cabinet.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=Origin(xyz=(x, 0.140, -0.005)),
            material=black,
            name=f"rear_foot_{i}",
        )

    # Tick marks and small legends around the two stacked dials are raised from
    # the same fixed front-panel part, so they read as printed control details.
    knob_x = 0.225
    knob_zs = (0.245, 0.125)
    tick_y = panel_front_y - 0.0006
    for row, zc in enumerate(knob_zs):
        tick_specs = (
            (0.000, 0.047, 0.006, 0.001, 0.018, 0.0),
            (0.000, -0.047, 0.006, 0.001, 0.018, 0.0),
            (-0.047, 0.000, 0.001, 0.006, 0.018, 0.0),
            (0.047, 0.000, 0.001, 0.006, 0.018, 0.0),
            (-0.033, 0.033, 0.001, 0.005, 0.016, math.radians(45.0)),
            (0.033, 0.033, 0.001, 0.005, 0.016, math.radians(-45.0)),
        )
        for mark, (dx, dz, sx, sz, depth, yaw) in enumerate(tick_specs):
            cabinet.visual(
                Box((sx, 0.0012, sz)),
                origin=Origin(xyz=(knob_x + dx, tick_y, zc + dz), rpy=(0.0, 0.0, yaw)),
                material=black,
                name=f"tick_{row}_{mark}",
            )
        cabinet.visual(
            Box((0.078, 0.0012, 0.008)),
            origin=Origin(xyz=(knob_x, tick_y, zc - 0.075)),
            material=black,
            name=f"dial_label_{row}",
        )

    # Fixed hinge knuckles on the cabinet side of the vertical pin line.
    hinge_x = -0.325
    hinge_y = -0.230
    hinge_z = body_center_z
    for name, zc in (("hinge_knuckle_0", 0.078), ("hinge_knuckle_1", 0.282)):
        cabinet.visual(
            Cylinder(radius=0.010, length=0.076),
            origin=Origin(xyz=(hinge_x, hinge_y, zc)),
            material=brass,
            name=name,
        )
        cabinet.visual(
            Box((0.026, 0.034, 0.066)),
            origin=Origin(xyz=(hinge_x - 0.006, hinge_y + 0.015, zc)),
            material=brass,
            name=f"{name}_leaf",
        )

    door = model.part("door")
    door_w, door_h, door_t = 0.405, 0.275, 0.036
    door_frame = BezelGeometry(
        opening_size=(0.300, 0.175),
        outer_size=(door_w, door_h),
        depth=door_t,
        opening_shape="rounded_rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.020,
        outer_corner_radius=0.028,
        face=BezelFace(style="radiused_step", front_lip=0.004, fillet=0.002),
    )
    door.visual(
        mesh_from_geometry(door_frame, "door_frame"),
        origin=Origin(xyz=(0.2225, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="door_frame",
    )
    door.visual(
        Box((0.292, 0.007, 0.166)),
        origin=Origin(xyz=(0.2225, -0.023, 0.0)),
        material=dark_glass,
        name="glass_window",
    )
    door.visual(
        Cylinder(radius=0.0105, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brass,
        name="hinge_knuckle",
    )
    door.visual(
        Box((0.026, 0.014, 0.098)),
        origin=Origin(xyz=(0.014, 0.006, 0.0)),
        material=brass,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.205),
        origin=Origin(xyz=(0.385, -0.070, 0.0)),
        material=chrome,
        name="vertical_handle",
    )
    for name, zc in (("handle_mount_0", -0.074), ("handle_mount_1", 0.074)):
        door.visual(
            Cylinder(radius=0.0075, length=0.050),
            origin=Origin(xyz=(0.385, -0.044, zc), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=name,
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.032,
            body_style="skirted",
            top_diameter=0.058,
            base_diameter=0.074,
            edge_radius=0.0015,
            skirt=KnobSkirt(0.082, 0.006, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=20, depth=0.0016),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "retro_dial_knob",
    )

    for name, zc in (("upper_knob", knob_zs[0]), ("lower_knob", knob_zs[1])):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.012, length=0.014),
            origin=Origin(xyz=(0.0, -0.007, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="shaft",
        )
        knob.visual(
            knob_mesh,
            origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="cap",
        )
        model.articulation(
            f"{name}_shaft",
            ArticulationType.REVOLUTE,
            parent=cabinet,
            child=knob,
            origin=Origin(xyz=(knob_x, panel_front_y, zc)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=5.0, lower=-2.6, upper=2.6),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    upper_knob = object_model.get_part("upper_knob")
    lower_knob = object_model.get_part("lower_knob")
    door_hinge = object_model.get_articulation("door_hinge")
    upper_joint = object_model.get_articulation("upper_knob_shaft")
    lower_joint = object_model.get_articulation("lower_knob_shaft")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        min_gap=0.0,
        max_gap=0.010,
        positive_elem="rounded_cabinet",
        negative_elem="door_frame",
        name="closed door stands just proud of cabinet front",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="glass_window",
        elem_b="oven_cavity_shadow",
        min_overlap=0.150,
        name="wide glazed door covers the dark oven opening",
    )
    for knob, label in ((upper_knob, "upper"), (lower_knob, "lower")):
        ctx.expect_gap(
            cabinet,
            knob,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0005,
            positive_elem="control_panel",
            negative_elem="shaft",
            name=f"{label} knob shaft seats on control panel",
        )

    rest_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    with ctx.pose({door_hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")
    ctx.check(
        "door hinge swings the free edge outward",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][1] < rest_aabb[0][1] - 0.10,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    ctx.check(
        "control knobs use front-facing rotary shafts",
        tuple(round(v, 3) for v in upper_joint.axis) == (0.0, -1.0, 0.0)
        and tuple(round(v, 3) for v in lower_joint.axis) == (0.0, -1.0, 0.0),
        details=f"upper_axis={upper_joint.axis}, lower_axis={lower_joint.axis}",
    )

    return ctx.report()


object_model = build_object_model()
