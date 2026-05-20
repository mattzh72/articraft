from __future__ import annotations

import math

import cadquery as cq
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
)


def _add_quad(mesh: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    mesh.add_face(a, b, c)
    mesh.add_face(a, c, d)


def _arched_lid_shell(
    *,
    length: float,
    width: float,
    height: float,
    thickness: float,
    base_z: float,
    segments: int = 32,
) -> MeshGeometry:
    """Thin, open-bottom arched grill lid with the hinge at local x=0."""

    mesh = MeshGeometry()
    half_width = width / 2.0
    outer_radius_x = length / 2.0
    inner_radius_x = outer_radius_x - thickness
    inner_height = height - thickness

    outer_minus: list[int] = []
    outer_plus: list[int] = []
    inner_minus: list[int] = []
    inner_plus: list[int] = []

    for i in range(segments + 1):
        theta = math.pi - (math.pi * i / segments)
        outer_x = outer_radius_x + outer_radius_x * math.cos(theta)
        outer_z = base_z + height * math.sin(theta)
        inner_x = outer_radius_x + inner_radius_x * math.cos(theta)
        inner_z = base_z + inner_height * math.sin(theta)

        outer_minus.append(mesh.add_vertex(outer_x, -half_width, outer_z))
        outer_plus.append(mesh.add_vertex(outer_x, half_width, outer_z))
        inner_minus.append(mesh.add_vertex(inner_x, -half_width, inner_z))
        inner_plus.append(mesh.add_vertex(inner_x, half_width, inner_z))

    for i in range(segments):
        # Outside crown and inside hollow surface.
        _add_quad(mesh, outer_minus[i], outer_minus[i + 1], outer_plus[i + 1], outer_plus[i])
        _add_quad(mesh, inner_plus[i], inner_plus[i + 1], inner_minus[i + 1], inner_minus[i])

        # Thin side faces of the shell band at the two lid ends.
        _add_quad(mesh, outer_minus[i], inner_minus[i], inner_minus[i + 1], outer_minus[i + 1])
        _add_quad(mesh, outer_plus[i + 1], inner_plus[i + 1], inner_plus[i], outer_plus[i])

    # Close the lower front and rear lips.
    _add_quad(mesh, outer_minus[0], outer_plus[0], inner_plus[0], inner_minus[0])
    _add_quad(mesh, outer_minus[-1], inner_minus[-1], inner_plus[-1], outer_plus[-1])
    return mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_balcony_grill")

    enamel = model.material("matte_black_enamel", rgba=(0.02, 0.022, 0.020, 1.0))
    warm_lid = model.material("deep_red_lid", rgba=(0.42, 0.035, 0.025, 1.0))
    dark_metal = model.material("dark_powdercoat", rgba=(0.05, 0.055, 0.052, 1.0))
    grate_metal = model.material("brushed_steel", rgba=(0.58, 0.56, 0.50, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    body = model.part("body")

    # Compact firebox, built as an open tub rather than a solid block.
    body.visual(
        Box((0.46, 0.58, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.6475)),
        material=enamel,
        name="firebox_floor",
    )
    body.visual(
        Box((0.46, 0.035, 0.155)),
        origin=Origin(xyz=(0.0, 0.2725, 0.725)),
        material=enamel,
        name="side_panel",
    )
    body.visual(
        Box((0.46, 0.035, 0.155)),
        origin=Origin(xyz=(0.0, -0.2725, 0.725)),
        material=enamel,
        name="side_panel_1",
    )
    body.visual(
        Box((0.035, 0.58, 0.155)),
        origin=Origin(xyz=(0.2125, 0.0, 0.725)),
        material=enamel,
        name="front_wall",
    )
    body.visual(
        Box((0.035, 0.58, 0.155)),
        origin=Origin(xyz=(-0.2125, 0.0, 0.725)),
        material=enamel,
        name="rear_wall",
    )

    # Narrow balcony-style pedestal stand with a small stabilizing foot.
    body.visual(
        Box((0.12, 0.22, 0.625)),
        origin=Origin(xyz=(0.0, 0.0, 0.3225)),
        material=dark_metal,
        name="narrow_stand",
    )
    body.visual(
        Box((0.42, 0.10, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_metal,
        name="stand_foot_long",
    )
    body.visual(
        Box((0.10, 0.42, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=dark_metal,
        name="stand_foot_cross",
    )
    body.visual(
        Box((0.21, 0.31, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        material=dark_metal,
        name="lower_shelf",
    )

    # Cooking grate with side rails so the individual rods read as a supported assembly.
    body.visual(
        Box((0.385, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.205, 0.786)),
        material=grate_metal,
        name="grate_rail",
    )
    body.visual(
        Box((0.385, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.205, 0.786)),
        material=grate_metal,
        name="grate_rail_1",
    )
    body.visual(
        Box((0.385, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, 0.232, 0.786)),
        material=dark_metal,
        name="grate_ledge",
    )
    body.visual(
        Box((0.385, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, -0.232, 0.786)),
        material=dark_metal,
        name="grate_ledge_1",
    )
    for idx, x in enumerate((-0.150, -0.100, -0.050, 0.0, 0.050, 0.100, 0.150)):
        body.visual(
            Cylinder(radius=0.004, length=0.430),
            origin=Origin(xyz=(x, 0.0, 0.790), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grate_metal,
            name=f"grate_bar_{idx}",
        )

    # Rear hinge support plates are fixed to the firebox and sit just under the lid hinge barrel.
    for suffix, y in (("0", -0.185), ("1", 0.185)):
        body.visual(
            Box((0.040, 0.060, 0.022)),
            origin=Origin(xyz=(-0.244, y, 0.797)),
            material=dark_metal,
            name=f"rear_hinge_bracket_{suffix}",
        )

    body.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.080, 0.296, 0.720), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grate_metal,
        name="control_bushing",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            _arched_lid_shell(
                length=0.475,
                width=0.610,
                height=0.175,
                thickness=0.018,
                base_z=-0.012,
            ),
            "rounded_lid_shell",
        ),
        material=warm_lid,
        name="rounded_shell",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(0.390, 0.0, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_handle",
    )
    for suffix, y in (("0", -0.150), ("1", 0.150)):
        lid.visual(
            Box((0.085, 0.022, 0.034)),
            origin=Origin(xyz=(0.355, y, 0.128)),
            material=dark_metal,
            name=f"handle_standoff_{suffix}",
        )

    control_knob = model.part("control_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.030,
            body_style="skirted",
            top_diameter=0.055,
            edge_radius=0.0015,
            grip=KnobGrip(style="fluted", count=18, depth=0.0015),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "side_temperature_knob",
    )
    control_knob.visual(
        Cylinder(radius=0.007, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=grate_metal,
        name="shaft",
    )
    control_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=rubber,
        name="knob_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.245, 0.0, 0.820)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "body_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=control_knob,
        origin=Origin(xyz=(0.080, 0.302, 0.720), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=-2.35, upper=2.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    knob = object_model.get_part("control_knob")
    lid_joint = object_model.get_articulation("body_to_lid")
    knob_joint = object_model.get_articulation("body_to_control_knob")

    with ctx.pose({lid_joint: 0.0, knob_joint: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.020,
            max_penetration=0.001,
            name="closed lid sits just above firebox rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.38,
            name="closed lid covers compact cook body",
        )
        ctx.expect_gap(
            knob,
            body,
            axis="y",
            positive_elem="shaft",
            negative_elem="control_bushing",
            max_gap=0.0015,
            max_penetration=0.0,
            name="knob shaft seats at side bushing",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="xz",
            elem_a="shaft",
            elem_b="control_bushing",
            min_overlap=0.010,
            name="knob shaft is centered on side bushing",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_joint: 1.15}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "rear hinge opens lid upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )
    ctx.check(
        "side knob has realistic rotary travel",
        knob_joint.motion_limits is not None
        and knob_joint.motion_limits.lower <= -2.0
        and knob_joint.motion_limits.upper >= 2.0,
        details=f"limits={knob_joint.motion_limits}",
    )

    return ctx.report()


object_model = build_object_model()
