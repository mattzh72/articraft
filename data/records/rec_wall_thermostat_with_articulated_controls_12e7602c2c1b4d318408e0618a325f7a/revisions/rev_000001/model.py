from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)
import cadquery as cq


def _rounded_rect_prism(width: float, height: float, depth: float, radius: float, *, segments: int = 8) -> MeshGeometry:
    """Rounded-rectangle slab with depth along local +X and face in local YZ."""
    profile = rounded_rect_profile(width, height, radius, corner_segments=segments)
    geom = MeshGeometry()

    back: list[int] = []
    front: list[int] = []
    for y, z in profile:
        back.append(geom.add_vertex(-depth / 2.0, y, z))
        front.append(geom.add_vertex(depth / 2.0, y, z))

    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back[i], back[j], front[j])
        geom.add_face(back[i], front[j], front[i])

    back_center = geom.add_vertex(-depth / 2.0, 0.0, 0.0)
    front_center = geom.add_vertex(depth / 2.0, 0.0, 0.0)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(back_center, back[i], back[j])
        geom.add_face(front_center, front[j], front[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_wall_thermostat")

    warm_white = model.material("warm_white", color=(0.86, 0.84, 0.78, 1.0))
    soft_white = model.material("soft_white", color=(0.93, 0.92, 0.87, 1.0))
    dark_plastic = model.material("dark_plastic", color=(0.035, 0.038, 0.042, 1.0))
    smoked_glass = model.material("smoked_glass", color=(0.02, 0.05, 0.06, 0.86))
    lcd_green = model.material("lcd_green", color=(0.45, 0.75, 0.62, 1.0))
    pale_gray = model.material("pale_gray", color=(0.72, 0.72, 0.68, 1.0))
    satin_metal = model.material("satin_metal", color=(0.55, 0.55, 0.52, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_rounded_rect_prism(0.118, 0.118, 0.024, 0.015), "body_shell"),
        material=warm_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_geometry(_rounded_rect_prism(0.106, 0.106, 0.0015, 0.012), "front_plate"),
        origin=Origin(xyz=(0.01275, 0.0, 0.0)),
        material=soft_white,
        name="front_plate",
    )
    body.visual(
        Box((0.0010, 0.074, 0.029)),
        origin=Origin(xyz=(0.0140, 0.0, 0.038)),
        material=dark_plastic,
        name="display_bezel",
    )
    body.visual(
        Box((0.0012, 0.063, 0.019)),
        origin=Origin(xyz=(0.01475, 0.0, 0.038)),
        material=smoked_glass,
        name="display_glass",
    )
    body.visual(
        Box((0.0010, 0.036, 0.005)),
        origin=Origin(xyz=(0.0154, -0.010, 0.040)),
        material=lcd_green,
        name="temperature_digits",
    )
    body.visual(
        Box((0.0008, 0.006, 0.006)),
        origin=Origin(xyz=(0.0155, 0.025, 0.040)),
        material=lcd_green,
        name="degree_mark",
    )
    body.visual(
        Box((0.0009, 0.087, 0.032)),
        origin=Origin(xyz=(0.0136, 0.0, -0.040)),
        material=dark_plastic,
        name="control_recess",
    )
    body.visual(
        Box((0.0040, 0.092, 0.0100)),
        origin=Origin(xyz=(0.0148, 0.0, -0.0555)),
        material=satin_metal,
        name="hinge_leaf",
    )
    for i, y in enumerate((-0.034, 0.034)):
        body.visual(
            Cylinder(radius=0.0022, length=0.018),
            origin=Origin(xyz=(0.0170, y, -0.0575), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name=f"hinge_knuckle_{i}",
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.049,
                0.011,
                body_style="domed",
                base_diameter=0.052,
                top_diameter=0.044,
                crown_radius=0.0015,
                edge_radius=0.0012,
                grip=KnobGrip(style="ribbed", count=36, depth=0.00065, width=0.0010),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "dial_cap",
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pale_gray,
        name="dial_cap",
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0140, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    panel = model.part("button_panel")
    panel.visual(
        mesh_from_geometry(_rounded_rect_prism(0.084, 0.030, 0.0030, 0.004), "panel_cover"),
        origin=Origin(xyz=(0.0015, 0.0, 0.0185)),
        material=soft_white,
        name="panel_cover",
    )
    panel.visual(
        Box((0.0009, 0.038, 0.0020)),
        origin=Origin(xyz=(0.0032, 0.0, 0.0090)),
        material=pale_gray,
        name="finger_groove",
    )
    panel.visual(
        Cylinder(radius=0.0021, length=0.044),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="panel_knuckle",
    )
    panel.visual(
        Box((0.0020, 0.050, 0.0040)),
        origin=Origin(xyz=(0.0010, 0.0, 0.0035)),
        material=satin_metal,
        name="panel_hinge_leaf",
    )
    model.articulation(
        "panel_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=panel,
        origin=Origin(xyz=(0.0170, 0.0, -0.0575)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.2, lower=0.0, upper=1.75),
    )

    for i, y in enumerate((-0.026, 0.0, 0.026)):
        button = model.part(f"button_{i}")
        button.visual(
            mesh_from_geometry(_rounded_rect_prism(0.018, 0.008, 0.0016, 0.002), f"button_cap_{i}"),
            origin=Origin(xyz=(0.0008, 0.0, 0.0)),
            material=pale_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.0008, 0.010, 0.004)),
            origin=Origin(xyz=(-0.0003, 0.0, 0.0)),
            material=dark_plastic,
            name="button_stem",
        )
        model.articulation(
            f"button_press_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.0146, y, -0.0415)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=0.04, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    dial = object_model.get_part("dial")
    panel = object_model.get_part("button_panel")
    dial_joint = object_model.get_articulation("dial_turn")
    panel_joint = object_model.get_articulation("panel_hinge")

    ctx.check(
        "front dial is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_joint.axis) == (1.0, 0.0, 0.0),
        details=f"type={dial_joint.articulation_type}, axis={dial_joint.axis}",
    )
    ctx.check(
        "button panel is lower-hinged",
        panel_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(panel_joint.axis) == (0.0, 1.0, 0.0)
        and panel_joint.motion_limits is not None
        and panel_joint.motion_limits.lower == 0.0
        and panel_joint.motion_limits.upper >= 1.5,
        details=f"type={panel_joint.articulation_type}, axis={panel_joint.axis}, limits={panel_joint.motion_limits}",
    )
    ctx.expect_gap(
        dial,
        body,
        axis="x",
        min_gap=0.0001,
        max_gap=0.004,
        positive_elem="dial_cap",
        negative_elem="front_plate",
        name="dial sits proud of front plate",
    )
    ctx.expect_gap(
        panel,
        object_model.get_part("button_1"),
        axis="x",
        min_gap=0.0002,
        max_gap=0.0025,
        positive_elem="panel_cover",
        negative_elem="button_cap",
        name="closed cover clears hidden buttons",
    )

    closed_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_joint: 1.55}):
        open_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "panel flips down and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.015
        and open_aabb[1][2] < closed_aabb[1][2] - 0.010,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
