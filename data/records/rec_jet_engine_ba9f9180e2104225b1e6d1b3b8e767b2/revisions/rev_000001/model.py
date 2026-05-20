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
    FanRotorShroud,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _petal_panel(length: float, root_width: float, tip_width: float, thickness: float) -> MeshGeometry:
    """Trapezoidal nozzle flap with its hinge line on local X=0."""
    g = MeshGeometry()
    wr = root_width / 2.0
    wt = tip_width / 2.0
    verts = [
        (0.0, -wr, 0.0),
        (0.0, wr, 0.0),
        (length, -wt, 0.0),
        (length, wt, 0.0),
        (0.0, -wr, thickness),
        (0.0, wr, thickness),
        (length, -wt, thickness),
        (length, wt, thickness),
    ]
    for v in verts:
        g.add_vertex(*v)
    for face in (
        (0, 2, 1),
        (1, 2, 3),
        (4, 5, 6),
        (5, 7, 6),
        (0, 1, 4),
        (1, 5, 4),
        (2, 6, 3),
        (3, 6, 7),
        (0, 4, 2),
        (2, 4, 6),
        (1, 3, 5),
        (3, 7, 5),
    ):
        g.add_face(*face)
    return g


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="military_turbofan")

    model.material("gunmetal", rgba=(0.23, 0.25, 0.24, 1.0))
    model.material("dark_oxide", rgba=(0.045, 0.050, 0.050, 1.0))
    model.material("burnt_titanium", rgba=(0.46, 0.38, 0.30, 1.0))
    model.material("matte_black", rgba=(0.006, 0.007, 0.008, 1.0))
    model.material("olive_panel", rgba=(0.28, 0.31, 0.22, 1.0))
    model.material("warning_red", rgba=(0.75, 0.12, 0.05, 1.0))
    model.material("fan_blade", rgba=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")

    # A thin-walled, open-ended engine casing.  The profile is revolved about
    # local Z and then turned so the engine's main axis is world X.
    body_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.430, -1.48),
            (0.470, -1.32),
            (0.455, -0.60),
            (0.405, 0.45),
            (0.370, 1.18),
        ],
        inner_profile=[
            (0.350, -1.48),
            (0.375, -1.32),
            (0.355, -0.60),
            (0.315, 0.45),
            (0.285, 1.18),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(body_shell, "body_shell"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="gunmetal",
        name="body_shell",
    )

    # Raised reinforcing bands and rear hinge ring give the engine a military
    # hot-section silhouette and make the aft nozzle hardware read as mounted.
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.462, tube=0.018, radial_segments=18, tubular_segments=96), "case_band_0"),
        origin=Origin(xyz=(-1.36, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_oxide",
        name="case_band_0",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.452, tube=0.010, radial_segments=18, tubular_segments=96), "case_band_1"),
        origin=Origin(xyz=(-0.58, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_oxide",
        name="case_band_1",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.402, tube=0.010, radial_segments=18, tubular_segments=96), "case_band_2"),
        origin=Origin(xyz=(0.52, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_oxide",
        name="case_band_2",
    )
    body.visual(
        mesh_from_geometry(TorusGeometry(radius=0.365, tube=0.018, radial_segments=18, tubular_segments=96), "case_band_3"),
        origin=Origin(xyz=(1.16, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="burnt_titanium",
        name="case_band_3",
    )

    # Dark exhaust liner visible through the opened petals.
    exhaust_liner = LatheGeometry.from_shell_profiles(
        outer_profile=[(0.300, 0.38), (0.285, 1.20)],
        inner_profile=[(0.255, 0.38), (0.240, 1.20)],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    body.visual(
        mesh_from_geometry(exhaust_liner, "exhaust_liner"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="matte_black",
        name="exhaust_liner",
    )

    # Fixed front bearing and cross-shaped stator support for the rotating fan.
    body.visual(
        Cylinder(radius=0.055, length=0.14),
        origin=Origin(xyz=(-1.14, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_oxide",
        name="front_bearing",
    )
    for name, xyz, size in (
        ("stator_y_pos", (-1.14, 0.218, 0.0), (0.050, 0.326, 0.026)),
        ("stator_y_neg", (-1.14, -0.218, 0.0), (0.050, 0.326, 0.026)),
        ("stator_z_pos", (-1.14, 0.0, 0.218), (0.050, 0.026, 0.326)),
        ("stator_z_neg", (-1.14, 0.0, -0.218), (0.050, 0.026, 0.326)),
    ):
        body.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material="dark_oxide",
            name=name,
        )

    # Equipment bay recess on the starboard side of the rear case.
    body.visual(
        Box((0.76, 0.024, 0.34)),
        origin=Origin(xyz=(0.72, 0.410, 0.04)),
        material="matte_black",
        name="bay_recess",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.39),
        origin=Origin(xyz=(0.36, 0.438, 0.04)),
        material="dark_oxide",
        name="bay_hinge_barrel",
    )
    for z in (-0.16, 0.24):
        body.visual(
            Box((0.80, 0.030, 0.018)),
            origin=Origin(xyz=(0.73, 0.430, z)),
            material="dark_oxide",
            name=f"bay_frame_{'lower' if z < 0 else 'upper'}",
        )

    fan = model.part("front_fan")
    fan_rotor = FanRotorGeometry(
        outer_radius=0.305,
        hub_radius=0.078,
        blade_count=18,
        thickness=0.075,
        blade_pitch_deg=34.0,
        blade_sweep_deg=38.0,
        blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=15.0, camber=0.12),
        hub=FanRotorHub(style="spinner", rear_collar_height=0.045, rear_collar_radius=0.068, bore_diameter=0.022),
        shroud=FanRotorShroud(thickness=0.010, depth=0.060, clearance=0.006, lip_depth=0.008),
    )
    fan.visual(
        mesh_from_geometry(fan_rotor, "front_fan_rotor"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="fan_blade",
        name="rotor",
    )
    fan.visual(
        Cylinder(radius=0.035, length=0.26),
        origin=Origin(xyz=(0.10, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_oxide",
        name="fan_shaft",
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=fan,
        origin=Origin(xyz=(-1.30, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=80.0),
    )

    bay_door = model.part("bay_door")
    bay_door.visual(
        Box((0.70, 0.028, 0.32)),
        origin=Origin(xyz=(0.35, 0.0, 0.0)),
        material="olive_panel",
        name="door_skin",
    )
    bay_door.visual(
        Box((0.66, 0.014, 0.035)),
        origin=Origin(xyz=(0.36, 0.020, 0.115)),
        material="dark_oxide",
        name="stiffener_upper",
    )
    bay_door.visual(
        Box((0.66, 0.014, 0.035)),
        origin=Origin(xyz=(0.36, 0.020, -0.115)),
        material="dark_oxide",
        name="stiffener_lower",
    )
    bay_door.visual(
        Cylinder(radius=0.012, length=0.33),
        origin=Origin(xyz=(0.00, 0.013, 0.0)),
        material="dark_oxide",
        name="door_hinge_pin",
    )
    bay_door.visual(
        Box((0.055, 0.010, 0.018)),
        origin=Origin(xyz=(0.62, 0.014, 0.0)),
        material="warning_red",
        name="pull_latch",
    )
    model.articulation(
        "bay_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=bay_door,
        origin=Origin(xyz=(0.36, 0.455, 0.04)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.8, lower=0.0, upper=1.25),
    )

    # Eight independent variable-area nozzle petals, already posed open at q=0
    # but still rotating about their aft case hinges.
    petal_count = 8
    hinge_x = 1.16
    hinge_radius = 0.355
    petal_length = 0.52
    open_angle = 0.32
    for i in range(petal_count):
        phi = 2.0 * math.pi * i / petal_count
        radial_y = math.cos(phi)
        radial_z = math.sin(phi)
        petal = model.part(f"nozzle_petal_{i}")
        root_width = 2.0 * math.pi * hinge_radius / petal_count * 0.72
        tip_width = root_width * 0.82
        petal.visual(
            mesh_from_geometry(_petal_panel(petal_length, root_width, tip_width, 0.022), f"nozzle_petal_panel_{i}"),
            origin=Origin(rpy=(0.0, -open_angle, 0.0)),
            material="burnt_titanium",
            name="petal_skin",
        )
        petal.visual(
            Cylinder(radius=0.018, length=root_width * 0.74),
            origin=Origin(xyz=(0.0, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="dark_oxide",
            name="hinge_knuckle",
        )
        petal.visual(
            Box((petal_length * 0.70, 0.022, 0.018)),
            origin=Origin(
                xyz=(0.30 * math.cos(open_angle), 0.0, 0.30 * math.sin(open_angle) + 0.030),
                rpy=(0.0, -open_angle, 0.0),
            ),
            material="dark_oxide",
            name="center_rib",
        )
        model.articulation(
            f"nozzle_petal_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=petal,
            origin=Origin(
                xyz=(hinge_x, hinge_radius * radial_y, hinge_radius * radial_z),
                rpy=(phi - math.pi / 2.0, 0.0, 0.0),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.22, upper=0.42),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    fan = object_model.get_part("front_fan")
    bay_door = object_model.get_part("bay_door")
    fan_spin = object_model.get_articulation("fan_spin")
    door_hinge = object_model.get_articulation("bay_door_hinge")

    ctx.allow_overlap(
        body,
        fan,
        elem_a="front_bearing",
        elem_b="fan_shaft",
        reason="The rotating fan shaft is intentionally captured inside the fixed front bearing.",
    )
    ctx.expect_within(
        fan,
        body,
        axes="yz",
        inner_elem="fan_shaft",
        outer_elem="front_bearing",
        margin=0.0,
        name="fan shaft is centered in the front bearing",
    )
    ctx.expect_overlap(
        fan,
        body,
        axes="x",
        elem_a="fan_shaft",
        elem_b="front_bearing",
        min_overlap=0.08,
        name="fan shaft remains inserted through the front bearing",
    )

    ctx.expect_within(
        fan,
        body,
        axes="yz",
        inner_elem="rotor",
        outer_elem="body_shell",
        margin=0.08,
        name="front fan sits inside the open inlet",
    )
    ctx.expect_gap(
        bay_door,
        body,
        axis="y",
        positive_elem="door_skin",
        negative_elem="bay_recess",
        min_gap=0.015,
        max_gap=0.075,
        name="bay door is proud of the side recess",
    )
    ctx.allow_overlap(
        bay_door,
        body,
        elem_a="door_skin",
        elem_b="bay_hinge_barrel",
        reason="The folded hinge edge of the equipment bay door wraps around the fixed hinge barrel.",
    )
    ctx.expect_gap(
        bay_door,
        body,
        axis="y",
        positive_elem="door_skin",
        negative_elem="bay_hinge_barrel",
        max_penetration=0.020,
        name="bay door hinge edge is seated on the hinge barrel",
    )

    rest_door = ctx.part_element_world_aabb(bay_door, elem="door_skin")
    with ctx.pose({door_hinge: 1.0}):
        open_door = ctx.part_element_world_aabb(bay_door, elem="door_skin")
    ctx.check(
        "bay door swings outward",
        rest_door is not None and open_door is not None and open_door[1][1] > rest_door[1][1] + 0.10,
        details=f"rest={rest_door}, open={open_door}",
    )

    rest_fan = ctx.part_world_position(fan)
    with ctx.pose({fan_spin: math.pi / 2.0}):
        spun_fan = ctx.part_world_position(fan)
    ctx.check(
        "fan spins about fixed main axis",
        rest_fan is not None and spun_fan is not None and abs(spun_fan[0] - rest_fan[0]) < 0.001,
        details=f"rest={rest_fan}, spun={spun_fan}",
    )

    for i in range(8):
        petal_i = object_model.get_part(f"nozzle_petal_{i}")
        ctx.allow_overlap(
            body,
            petal_i,
            elem_a="body_shell",
            elem_b="hinge_knuckle",
            reason="Each nozzle petal hinge knuckle is seated into a rear case hinge lug.",
        )
        ctx.expect_overlap(
            petal_i,
            body,
            axes="x",
            elem_a="hinge_knuckle",
            elem_b="body_shell",
            min_overlap=0.015,
            name=f"nozzle petal {i} hinge is retained by the rear case",
        )
        ctx.allow_overlap(
            body,
            petal_i,
            elem_a="body_shell",
            elem_b="petal_skin",
            reason="The root lip of each opened nozzle petal tucks under the aft casing edge.",
        )
        ctx.expect_overlap(
            petal_i,
            body,
            axes="x",
            elem_a="petal_skin",
            elem_b="body_shell",
            min_overlap=0.015,
            name=f"nozzle petal {i} root lip stays captured by the casing",
        )
        ctx.allow_overlap(
            body,
            petal_i,
            elem_a="case_band_3",
            elem_b="hinge_knuckle",
            reason="The rear nozzle hinge knuckle is intentionally nested through the external hinge ring.",
        )
        ctx.expect_overlap(
            petal_i,
            body,
            axes="x",
            elem_a="hinge_knuckle",
            elem_b="case_band_3",
            min_overlap=0.015,
            name=f"nozzle petal {i} hinge crosses the rear hinge ring",
        )
        ctx.allow_overlap(
            body,
            petal_i,
            elem_a="case_band_3",
            elem_b="petal_skin",
            reason="The external rear hinge ring overlaps the petal root lip as a captured retaining collar.",
        )
        ctx.expect_overlap(
            petal_i,
            body,
            axes="x",
            elem_a="petal_skin",
            elem_b="case_band_3",
            min_overlap=0.015,
            name=f"nozzle petal {i} root lip is captured by the hinge ring",
        )

    petal = object_model.get_part("nozzle_petal_0")
    petal_hinge = object_model.get_articulation("nozzle_petal_hinge_0")
    rest_petal_aabb = ctx.part_world_aabb(petal)
    with ctx.pose({petal_hinge: 0.35}):
        open_petal_aabb = ctx.part_world_aabb(petal)
    ctx.check(
        "nozzle petal rotates farther open",
        rest_petal_aabb is not None
        and open_petal_aabb is not None
        and open_petal_aabb[1][1] > rest_petal_aabb[1][1] + 0.03,
        details=f"rest={rest_petal_aabb}, open={open_petal_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
