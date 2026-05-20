from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded box in meters, centered at the local origin."""
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape


def _tower_shell_mesh() -> cq.Workplane:
    """One fused plastic shell: side rails, rear wall, and top/bottom caps."""
    width = 0.190
    depth = 0.160
    height = 0.900
    side_wall = 0.018
    rear_wall = 0.018
    cap_h = 0.035
    z_min = 0.020
    z_mid = z_min + height / 2.0

    shell = _rounded_box((side_wall, depth, height), 0.006).translate(
        (-width / 2.0 + side_wall / 2.0, 0.0, z_mid)
    )
    shell = shell.union(
        _rounded_box((side_wall, depth, height), 0.006).translate(
            (width / 2.0 - side_wall / 2.0, 0.0, z_mid)
        )
    )
    shell = shell.union(
        _rounded_box((width, rear_wall, height), 0.006).translate(
            (0.0, depth / 2.0 - rear_wall / 2.0, z_mid)
        )
    )
    shell = shell.union(
        _rounded_box((width, depth, cap_h), 0.007).translate(
            (0.0, 0.0, z_min + cap_h / 2.0)
        )
    )
    shell = shell.union(
        _rounded_box((width, depth, cap_h), 0.007).translate(
            (0.0, 0.0, z_min + height - cap_h / 2.0)
        )
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_tower_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.86, 0.80, 1.0))
    dark_gray = model.material("dark_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    rubber = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    metal = model.material("brushed_metal", rgba=(0.58, 0.60, 0.60, 1.0))
    rotor_mat = model.material("smoked_rotor", rgba=(0.22, 0.25, 0.27, 1.0))

    base = model.part("pedestal")
    base.visual(
        Cylinder(radius=0.180, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=warm_white,
        name="oval_foot",
    )
    base.visual(
        Cylinder(radius=0.168, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="rubber_sole",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.130),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=warm_white,
        name="pedestal_neck",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.1825)),
        material=warm_white,
        name="turntable_collar",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_tower_shell_mesh(), "body_shell", tolerance=0.0008),
        material=warm_white,
        name="shell_frame",
    )
    body.visual(
        Cylinder(radius=0.064, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=warm_white,
        name="rotating_collar",
    )

    grille = SlotPatternPanelGeometry(
        (0.170, 0.790),
        0.004,
        slot_size=(0.710, 0.0032),
        pitch=(0.0105, 0.900),
        frame=0.010,
        corner_radius=0.004,
        slot_angle_deg=89.5,
    )
    body.visual(
        mesh_from_geometry(grille, "intake_grille"),
        origin=Origin(xyz=(0.0, -0.082, 0.460), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="intake_grille",
    )
    body.visual(
        Box((0.135, 0.070, 0.006)),
        origin=Origin(xyz=(0.0, -0.015, 0.923)),
        material=black,
        name="control_deck",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=metal,
        name="bottom_bearing",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=metal,
        name="top_bearing",
    )
    for x, visual_name in ((-0.025, "bearing_strut_0"), (0.025, "bearing_strut_1")):
        body.visual(
            Box((0.022, 0.068, 0.014)),
            origin=Origin(xyz=(x, 0.039, 0.840)),
            material=metal,
            name=visual_name,
        )

    model.articulation(
        "pedestal_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.7, lower=-0.85, upper=0.85),
    )

    blower = model.part("blower_wheel")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.052,
                inner_radius=0.026,
                width=0.680,
                blade_count=32,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "blower_wheel",
        ),
        material=rotor_mat,
        name="blower_cage",
    )
    blower.visual(
        Cylinder(radius=0.012, length=0.775),
        material=metal,
        name="blower_shaft",
    )
    blower.visual(
        Cylinder(radius=0.028, length=0.060),
        material=metal,
        name="blower_hub",
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, 0.452)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=70.0),
    )

    knob_geom = KnobGeometry(
        0.045,
        0.024,
        body_style="cylindrical",
        edge_radius=0.001,
        grip=KnobGrip(style="knurled", count=42, depth=0.0009, helix_angle_deg=18.0),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )

    for name, x in (("speed_knob", -0.040), ("timer_knob", 0.040)):
        knob = model.part(name)
        knob.visual(
            Cylinder(radius=0.008, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=metal,
            name="shaft",
        )
        knob.visual(
            mesh_from_geometry(knob_geom, f"{name}_cap"),
            origin=Origin(xyz=(0.0, 0.0, 0.018)),
            material=black,
            name="knurled_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(x, -0.015, 0.926)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-2.35, upper=2.35),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower_wheel")
    speed_knob = object_model.get_part("speed_knob")
    timer_knob = object_model.get_part("timer_knob")
    oscillation = object_model.get_articulation("pedestal_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")
    speed_joint = object_model.get_articulation("body_to_speed_knob")
    timer_joint = object_model.get_articulation("body_to_timer_knob")

    ctx.allow_overlap(
        body,
        blower,
        elem_a="bottom_bearing",
        elem_b="blower_shaft",
        reason="The rotating blower shaft is intentionally captured inside the lower bearing sleeve.",
    )
    ctx.allow_overlap(
        body,
        blower,
        elem_a="top_bearing",
        elem_b="blower_shaft",
        reason="The rotating blower shaft is intentionally captured inside the upper bearing sleeve.",
    )

    ctx.expect_gap(
        body,
        pedestal,
        axis="z",
        positive_elem="rotating_collar",
        negative_elem="turntable_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="oscillation collar sits on pedestal bearing",
    )
    ctx.expect_overlap(
        body,
        pedestal,
        axes="xy",
        elem_a="rotating_collar",
        elem_b="turntable_collar",
        min_overlap=0.10,
        name="oscillation collar is centered on pedestal",
    )

    for bearing_name in ("bottom_bearing", "top_bearing"):
        ctx.expect_within(
            blower,
            body,
            axes="xy",
            inner_elem="blower_shaft",
            outer_elem=bearing_name,
            name=f"blower shaft centered in {bearing_name}",
        )
        ctx.expect_overlap(
            blower,
            body,
            axes="z",
            elem_a="blower_shaft",
            elem_b=bearing_name,
            min_overlap=0.008,
            name=f"blower shaft retained by {bearing_name}",
        )

    ctx.expect_gap(
        blower,
        body,
        axis="y",
        positive_elem="blower_cage",
        negative_elem="intake_grille",
        min_gap=0.010,
        max_gap=0.050,
        name="blower clears the intake grille",
    )

    for knob, label in ((speed_knob, "speed"), (timer_knob, "timer")):
        ctx.expect_gap(
            knob,
            body,
            axis="z",
            positive_elem="shaft",
            negative_elem="control_deck",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"{label} knob shaft seats on control deck",
        )

    def _aabb_center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) / 2.0 if aabb is not None else None

    rest_grille = ctx.part_element_world_aabb(body, elem="intake_grille")
    rest_x = _aabb_center_x(rest_grille)
    with ctx.pose({oscillation: 0.85}):
        swept_grille = ctx.part_element_world_aabb(body, elem="intake_grille")
        swept_x = _aabb_center_x(swept_grille)
    ctx.check(
        "body oscillates left and right about vertical pedestal axis",
        rest_x is not None and swept_x is not None and swept_x > rest_x + 0.040,
        details=f"rest grille x={rest_x}, swept grille x={swept_x}",
    )

    ctx.check(
        "blower wheel has continuous vertical spin joint",
        blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(blower_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={blower_spin.articulation_type}, axis={blower_spin.axis}",
    )
    for joint, label in ((speed_joint, "speed"), (timer_joint, "timer")):
        limits = joint.motion_limits
        ctx.check(
            f"{label} knob rotates on its own short shaft",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.upper - limits.lower > 4.0,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
