from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    wire_from_points,
)


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius, length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius, length), Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_tilting_fan")

    cream = Material("aged_cream_enamel", rgba=(0.78, 0.72, 0.60, 1.0))
    dark = Material("charcoal_motor", rgba=(0.10, 0.11, 0.11, 1.0))
    steel = Material("brushed_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    brass = Material("dull_brass", rgba=(0.72, 0.55, 0.25, 1.0))
    rubber = Material("black_rubber", rgba=(0.025, 0.025, 0.022, 1.0))
    amber = Material("aged_amber_blades", rgba=(0.86, 0.60, 0.28, 1.0))

    # Root assembly: broad cast base, service hatch, column, and reinforced yoke.
    base = model.part("base")
    base.visual(Box((0.58, 0.42, 0.060)), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=cream, name="cast_foot")
    base.visual(Box((0.40, 0.27, 0.048)), origin=Origin(xyz=(0.0, -0.015, 0.082)), material=cream, name="raised_plinth")
    base.visual(Box((0.22, 0.135, 0.008)), origin=Origin(xyz=(0.0, 0.060, 0.110)), material=steel, name="base_service_hatch")
    for ix, sx in enumerate((-1.0, 1.0)):
        for iy, sy in enumerate((-1.0, 1.0)):
            base.visual(
                Cylinder(0.011, 0.007),
                origin=Origin(xyz=(sx * 0.083, 0.060 + sy * 0.045, 0.1175)),
                material=brass,
                name=f"hatch_screw_{ix}_{iy}",
            )

    base.visual(Cylinder(0.058, 0.620), origin=Origin(xyz=(0.0, -0.015, 0.410)), material=cream, name="ribbed_column")
    base.visual(Cylinder(0.074, 0.035), origin=Origin(xyz=(0.0, -0.015, 0.115)), material=steel, name="lower_column_collar")
    base.visual(Cylinder(0.070, 0.040), origin=Origin(xyz=(0.0, -0.015, 0.720)), material=steel, name="upper_column_collar")
    base.visual(Box((0.82, 0.090, 0.070)), origin=Origin(xyz=(0.0, -0.015, 0.730)), material=cream, name="yoke_crosshead")

    for sx, side in ((-1.0, "side_0"), (1.0, "side_1")):
        x = sx * 0.410
        # The yoke cheek is one piece from crosshead to pivot so the fan head is visibly supported.
        base.visual(Box((0.058, 0.105, 0.405)), origin=Origin(xyz=(x, -0.015, 0.880)), material=cream, name=f"{side}_yoke_cheek")
        base.visual(Box((0.050, 0.120, 0.155)), origin=Origin(xyz=(x, -0.015, 1.010)), material=steel, name=f"{side}_bolted_adapter")
        cyl, ori = _cyl_x(0.060, 0.020)
        base.visual(cyl, origin=Origin(xyz=(x - sx * 0.039, -0.015, 1.050), rpy=ori.rpy), material=steel, name=f"{side}_pivot_bushing")
        cyl, ori = _cyl_x(0.024, 0.018)
        base.visual(cyl, origin=Origin(xyz=(sx * 0.390, -0.015, 1.050), rpy=ori.rpy), material=brass, name=f"{side}_pivot_cap")
        # Diagonal strap makes the legacy yoke read as reinforced, not as a thin unsupported post.
        base.visual(
            Box((0.044, 0.080, 0.140)),
            origin=Origin(xyz=(sx * 0.185, -0.015, 0.720), rpy=(0.0, sx * 0.34, 0.0)),
            material=steel,
            name=f"{side}_gusset_strap",
        )
        for iz, z in enumerate((0.950, 1.080)):
            for yy, y in enumerate((-0.055, 0.025)):
                cyl, ori = _cyl_y(0.0085, 0.011)
                base.visual(
                    cyl,
                    origin=Origin(xyz=(x, y, z), rpy=ori.rpy),
                    material=brass,
                    name=f"{side}_adapter_bolt_{iz}_{yy}",
                )

    for ix, x in enumerate((-0.225, 0.225)):
        for iy, y in enumerate((-0.165, 0.165)):
            base.visual(Box((0.090, 0.060, 0.014)), origin=Origin(xyz=(x, y, -0.007)), material=rubber, name=f"rubber_foot_{ix}_{iy}")

    # Tilting head: stationary guard, motor shell, service hatch, side trunnions, and reinforcements.
    head = model.part("fan_head")
    for y, label in ((0.102, "front"), (-0.106, "rear")):
        for radius, ring in ((0.335, "outer"), (0.245, "middle"), (0.135, "inner")):
            ring_geom = TorusGeometry(radius, 0.0065 if ring != "outer" else 0.009, radial_segments=16, tubular_segments=72)
            ring_geom.rotate_x(-math.pi / 2.0)
            head.visual(mesh_from_geometry(ring_geom, f"{label}_{ring}_guard_ring"), origin=Origin(xyz=(0.0, y, 0.0)), material=steel, name=f"{label}_{ring}_guard_ring")

        for i in range(12):
            a = i * math.tau / 12.0
            start_r = 0.070
            end_r = 0.337
            spoke_geom = wire_from_points(
                [
                    (start_r * math.cos(a), y, start_r * math.sin(a)),
                    (end_r * math.cos(a), y, end_r * math.sin(a)),
                ],
                radius=0.0032,
                radial_segments=10,
                cap_ends=True,
            )
            head.visual(mesh_from_geometry(spoke_geom, f"{label}_guard_spoke_{i}"), material=steel, name=f"{label}_guard_spoke_{i}")

    # Four perimeter tie rods make the two guard halves a connected cage.
    for i, a in enumerate((math.radians(38), math.radians(142), math.radians(218), math.radians(322))):
        r = 0.335
        cyl, ori = _cyl_y(0.006, 0.218)
        head.visual(cyl, origin=Origin(xyz=(r * math.cos(a), -0.002, r * math.sin(a)), rpy=ori.rpy), material=steel, name=f"cage_tie_rod_{i}")

    cyl, ori = _cyl_y(0.110, 0.210)
    head.visual(cyl, origin=Origin(xyz=(0.0, -0.115, 0.0), rpy=ori.rpy), material=dark, name="motor_shell")
    cyl, ori = _cyl_y(0.078, 0.034)
    head.visual(cyl, origin=Origin(xyz=(0.0, -0.237, 0.0), rpy=ori.rpy), material=steel, name="rear_service_hatch")
    for a_i, a in enumerate((math.radians(45), math.radians(135), math.radians(225), math.radians(315))):
        cyl, ori = _cyl_y(0.007, 0.010)
        head.visual(cyl, origin=Origin(xyz=(0.060 * math.cos(a), -0.258, 0.060 * math.sin(a)), rpy=ori.rpy), material=brass, name=f"rear_hatch_screw_{a_i}")
    cyl, ori = _cyl_y(0.045, 0.024)
    head.visual(cyl, origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=ori.rpy), material=steel, name="bearing_boss")
    cyl, ori = _cyl_y(0.008, 0.048)
    head.visual(cyl, origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=ori.rpy), material=steel, name="stationary_hub")

    # Side trunnions sit just inside the yoke bushings; the flat faces provide the tilt bearing surface.
    for sx, side in ((-1.0, "side_0"), (1.0, "side_1")):
        cyl, ori = _cyl_x(0.066, 0.034)
        head.visual(cyl, origin=Origin(xyz=(sx * 0.366, -0.015, 0.0), rpy=ori.rpy), material=steel, name=f"{side}_trunnion")
        head.visual(Box((0.130, 0.066, 0.080)), origin=Origin(xyz=(sx * 0.285, -0.015, 0.0)), material=dark, name=f"{side}_motor_lug")
        head.visual(Box((0.215, 0.060, 0.060)), origin=Origin(xyz=(sx * 0.185, -0.015, 0.0)), material=dark, name=f"{side}_motor_strut")
        head.visual(Box((0.070, 0.050, 0.250)), origin=Origin(xyz=(sx * 0.250, -0.013, 0.000), rpy=(0.0, 0.0, sx * 0.58)), material=steel, name=f"{side}_guard_brace")
        for z in (-0.030, 0.030):
            cyl, ori = _cyl_x(0.0075, 0.010)
            head.visual(cyl, origin=Origin(xyz=(sx * 0.355, -0.015, z), rpy=ori.rpy), material=brass, name=f"{side}_trunnion_bolt_{z:+.2f}")

    # Rotor spins as its own child link; the mesh has a bored, domed hub and scimitar blades.
    rotor = model.part("rotor")
    rotor_geom = FanRotorGeometry(
        0.245,
        0.058,
        5,
        thickness=0.042,
        blade_pitch_deg=33.0,
        blade_sweep_deg=25.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.12, tip_clearance=0.012),
        hub=FanRotorHub(style="capped", rear_collar_height=0.020, rear_collar_radius=0.052, bore_diameter=0.018),
    )
    rotor.visual(
        mesh_from_geometry(rotor_geom, "rotor_blades"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=amber,
        name="blades_and_hub",
    )
    cyl, ori = _cyl_y(0.028, 0.020)
    rotor.visual(cyl, origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=ori.rpy), material=brass, name="front_hub_cap")

    model.articulation(
        "tilt_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(0.0, -0.015, 1.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.50, upper=0.50),
        motion_properties=MotionProperties(damping=0.35, friction=0.08),
    )
    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=42.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.01),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    head = object_model.get_part("fan_head")
    rotor = object_model.get_part("rotor")
    tilt = object_model.get_articulation("tilt_pivot")
    spin = object_model.get_articulation("rotor_spin")

    ctx.allow_overlap(
        base,
        head,
        elem_a="side_0_pivot_bushing",
        elem_b="side_0_trunnion",
        reason="The trunnion is intentionally seated inside the yoke bushing proxy for the tilt bearing.",
    )
    ctx.allow_overlap(
        base,
        head,
        elem_a="side_1_pivot_bushing",
        elem_b="side_1_trunnion",
        reason="The trunnion is intentionally seated inside the yoke bushing proxy for the tilt bearing.",
    )
    ctx.expect_contact(base, head, elem_a="side_0_pivot_bushing", elem_b="side_0_trunnion", contact_tol=0.004, name="one trunnion bears in yoke")
    ctx.expect_contact(base, head, elem_a="side_1_pivot_bushing", elem_b="side_1_trunnion", contact_tol=0.004, name="opposite trunnion bears in yoke")
    ctx.expect_within(rotor, head, axes="xz", inner_elem="blades_and_hub", outer_elem="front_outer_guard_ring", margin=0.020, name="rotor is inside the guard diameter")
    ctx.expect_gap(head, rotor, axis="y", positive_elem="front_outer_guard_ring", negative_elem="blades_and_hub", min_gap=0.020, name="front guard clears spinning rotor")

    rest_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: 0.40}):
        tilted_aabb = ctx.part_world_aabb(head)
        ctx.expect_within(rotor, head, axes="xz", inner_elem="blades_and_hub", outer_elem="front_outer_guard_ring", margin=0.045, name="rotor remains caged while tilted")
    ctx.check(
        "tilt raises front cage",
        rest_aabb is not None and tilted_aabb is not None and tilted_aabb[1][2] > rest_aabb[1][2] + 0.015,
        details=f"rest={rest_aabb}, tilted={tilted_aabb}",
    )
    with ctx.pose({spin: math.pi / 3.0}):
        ctx.expect_within(rotor, head, axes="xz", inner_elem="blades_and_hub", outer_elem="front_outer_guard_ring", margin=0.020, name="spinning pose stays within guard")

    return ctx.report()


object_model = build_object_model()
