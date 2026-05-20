from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoltPattern,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="towed_twin_rail_aa_launcher")

    olive = Material("mat_olive_drab", rgba=(0.23, 0.30, 0.16, 1.0))
    dark_olive = Material("mat_dark_olive", rgba=(0.11, 0.15, 0.09, 1.0))
    black = Material("mat_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    gunmetal = Material("mat_gunmetal", rgba=(0.20, 0.22, 0.22, 1.0))
    worn_steel = Material("mat_worn_steel", rgba=(0.46, 0.48, 0.45, 1.0))
    optic_glass = Material("mat_deep_glass", rgba=(0.05, 0.12, 0.14, 1.0))

    # Static towed chassis: a low box frame on a real wheel/axle pair with a
    # tow eye.  The chassis carries only the lower half of the slew bearing.
    base = model.part("base")
    base.visual(Box((1.20, 0.76, 0.16)), origin=Origin(xyz=(0.0, 0.0, 0.29)), material=olive, name="chassis_pan")
    base.visual(Box((0.26, 1.08, 0.12)), origin=Origin(xyz=(-0.36, 0.0, 0.31)), material=olive, name="axle_crossbox")
    base.visual(Cylinder(radius=0.045, length=1.58), origin=Origin(xyz=(-0.36, 0.0, 0.29), rpy=(math.pi / 2, 0.0, 0.0)), material=worn_steel, name="wheel_axle")

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.285,
            0.135,
            inner_radius=0.205,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.014, count=24, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.018, depth=0.006),),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.015, radius=0.004),
        ),
        "utility_trailer_tire",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.205,
            0.118,
            rim=WheelRim(inner_radius=0.135, flange_height=0.015, flange_thickness=0.006, bead_seat_depth=0.006),
            hub=WheelHub(
                radius=0.060,
                width=0.090,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.078, hole_diameter=0.008),
            ),
            face=WheelFace(dish_depth=0.012, front_inset=0.005, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.010, window_radius=0.030),
            bore=WheelBore(style="round", diameter=0.030),
        ),
        "pressed_steel_wheel",
    )
    for side, y in (("near", -0.82), ("far", 0.82)):
        base.visual(tire_mesh, origin=Origin(xyz=(-0.36, y, 0.285), rpy=(0.0, 0.0, math.pi / 2)), material=black, name=f"{side}_tire")
        base.visual(wheel_mesh, origin=Origin(xyz=(-0.36, y, 0.285), rpy=(0.0, 0.0, math.pi / 2)), material=dark_olive, name=f"{side}_wheel")

    base.visual(Box((1.05, 0.12, 0.10)), origin=Origin(xyz=(-1.05, 0.0, 0.28)), material=olive, name="drawbar")
    base.visual(Box((0.13, 0.10, 0.08)), origin=Origin(xyz=(-1.56, 0.0, 0.28)), material=olive, name="tow_shank")
    base.visual(Box((0.08, 0.055, 0.055)), origin=Origin(xyz=(-1.59, -0.076, 0.28)), material=olive, name="tow_lug_0")
    base.visual(Box((0.08, 0.055, 0.055)), origin=Origin(xyz=(-1.59, 0.076, 0.28)), material=olive, name="tow_lug_1")
    base.visual(
        mesh_from_geometry(TorusGeometry(0.095, 0.018, radial_segments=36, tubular_segments=10), "tow_eye_ring"),
        origin=Origin(xyz=(-1.62, 0.0, 0.28), rpy=(0.0, math.pi / 2, 0.0)),
        material=worn_steel,
        name="tow_eye",
    )
    base.visual(Box((0.08, 0.82, 0.08)), origin=Origin(xyz=(0.44, 0.0, 0.22)), material=olive, name="rear_outrigger")
    base.visual(Box((0.13, 0.22, 0.045)), origin=Origin(xyz=(0.44, -0.52, 0.18)), material=dark_olive, name="footpad_0")
    base.visual(Box((0.13, 0.22, 0.045)), origin=Origin(xyz=(0.44, 0.52, 0.18)), material=dark_olive, name="footpad_1")
    base.visual(
        mesh_from_geometry(TorusGeometry(0.40, 0.035, radial_segments=48, tubular_segments=12), "lower_slew_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=worn_steel,
        name="lower_slew_ring",
    )

    # Yawing turntable and elevation cradle.  Both side arms are part of the
    # rotating turret so the whole cradle slews on the central vertical bearing.
    turntable = model.part("turntable")
    turntable.visual(Cylinder(radius=0.47, length=0.080), origin=Origin(xyz=(0.0, 0.0, 0.040)), material=dark_olive, name="turntable_disk")
    turntable.visual(
        mesh_from_geometry(TorusGeometry(0.36, 0.028, radial_segments=48, tubular_segments=10), "upper_slew_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=worn_steel,
        name="upper_slew_ring",
    )
    turntable.visual(Cylinder(radius=0.22, length=0.24), origin=Origin(xyz=(0.0, 0.0, 0.20)), material=olive, name="pedestal")
    turntable.visual(Box((0.52, 0.94, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.36)), material=olive, name="cradle_bridge")
    for y, nm in ((-0.42, "side_arm_0"), (0.42, "side_arm_1")):
        turntable.visual(Box((0.46, 0.10, 0.66)), origin=Origin(xyz=(0.0, y, 0.61)), material=olive, name=nm)
        turntable.visual(Cylinder(radius=0.128, length=0.105), origin=Origin(xyz=(0.0, y, 0.71), rpy=(math.pi / 2, 0.0, 0.0)), material=gunmetal, name=f"{nm}_bushing")
    turntable.visual(Box((0.12, 0.84, 0.12)), origin=Origin(xyz=(-0.23, 0.0, 0.52)), material=dark_olive, name="rear_cradle_brace")
    turntable.visual(Box((0.12, 0.84, 0.08)), origin=Origin(xyz=(0.20, 0.0, 0.34)), material=dark_olive, name="front_cradle_brace")

    model.articulation(
        "base_to_turntable",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.6, lower=-math.pi, upper=math.pi),
    )

    # Pitching twin-rail pack.  The child frame is the trunnion axis; at q=0
    # the launch rails lie level along +X, and positive pitch elevates them.
    launch_pack = model.part("launch_pack")
    launch_pack.visual(Cylinder(radius=0.070, length=0.740), origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=worn_steel, name="trunnion")
    launch_pack.visual(Box((0.34, 0.38, 0.16)), origin=Origin(xyz=(0.10, 0.0, 0.060)), material=dark_olive, name="trunnion_block")
    launch_pack.visual(Box((2.55, 0.060, 0.065)), origin=Origin(xyz=(1.15, -0.145, 0.135)), material=olive, name="rail_0")
    launch_pack.visual(Box((2.55, 0.060, 0.065)), origin=Origin(xyz=(1.15, 0.145, 0.135)), material=olive, name="rail_1")
    launch_pack.visual(Box((0.080, 0.430, 0.095)), origin=Origin(xyz=(-0.12, 0.0, 0.125)), material=dark_olive, name="rear_crosshead")
    launch_pack.visual(Box((0.070, 0.410, 0.080)), origin=Origin(xyz=(0.95, 0.0, 0.125)), material=dark_olive, name="mid_crosshead")
    launch_pack.visual(Box((0.090, 0.430, 0.100)), origin=Origin(xyz=(2.38, 0.0, 0.130)), material=dark_olive, name="front_crosshead")
    launch_pack.visual(Box((2.20, 0.050, 0.040)), origin=Origin(xyz=(1.15, 0.0, 0.055)), material=gunmetal, name="center_spine")
    launch_pack.visual(Box((0.15, 0.18, 0.08)), origin=Origin(xyz=(2.41, -0.145, 0.145)), material=gunmetal, name="rail_stop_0")
    launch_pack.visual(Box((0.15, 0.18, 0.08)), origin=Origin(xyz=(2.41, 0.145, 0.145)), material=gunmetal, name="rail_stop_1")
    launch_pack.visual(Box((0.32, 0.060, 0.150)), origin=Origin(xyz=(0.66, -0.205, 0.165)), material=dark_olive, name="sight_mount")
    launch_pack.visual(Box((0.34, 0.115, 0.050)), origin=Origin(xyz=(0.66, -0.285, 0.205)), material=dark_olive, name="sight_bracket")
    launch_pack.visual(Box((0.38, 0.145, 0.18)), origin=Origin(xyz=(0.78, -0.380, 0.255)), material=olive, name="sight_body")
    launch_pack.visual(Cylinder(radius=0.047, length=0.060), origin=Origin(xyz=(1.00, -0.380, 0.255), rpy=(0.0, math.pi / 2, 0.0)), material=optic_glass, name="sight_lens")
    launch_pack.visual(Cylinder(radius=0.035, length=0.145), origin=Origin(xyz=(0.57, -0.380, 0.255), rpy=(0.0, math.pi / 2, 0.0)), material=gunmetal, name="sight_eyepiece")

    model.articulation(
        "turntable_to_launch_pack",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=launch_pack,
        origin=Origin(xyz=(0.0, 0.0, 0.71)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.5, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    launch_pack = object_model.get_part("launch_pack")
    yaw = object_model.get_articulation("base_to_turntable")
    pitch = object_model.get_articulation("turntable_to_launch_pack")

    ctx.expect_contact(
        base,
        turntable,
        elem_a="lower_slew_ring",
        elem_b="turntable_disk",
        contact_tol=0.002,
        name="turntable rides on the lower slew ring",
    )
    ctx.expect_within(
        turntable,
        base,
        axes="xy",
        inner_elem="turntable_disk",
        outer_elem="lower_slew_ring",
        margin=0.13,
        name="yawing disk remains centered on slew ring",
    )
    ctx.expect_gap(
        launch_pack,
        turntable,
        axis="y",
        positive_elem="trunnion",
        negative_elem="side_arm_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="trunnion end bears against one cradle arm",
    )
    ctx.expect_gap(
        turntable,
        launch_pack,
        axis="y",
        positive_elem="side_arm_1",
        negative_elem="trunnion",
        max_gap=0.001,
        max_penetration=0.0,
        name="trunnion end bears against the other cradle arm",
    )

    rest_aabb = ctx.part_element_world_aabb(launch_pack, elem="front_crosshead")
    with ctx.pose({pitch: 1.0}):
        raised_aabb = ctx.part_element_world_aabb(launch_pack, elem="front_crosshead")
    ctx.check(
        "positive pitch elevates rail pack",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[0][2] > rest_aabb[0][2] + 0.50,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    before = ctx.part_world_position(turntable)
    with ctx.pose({yaw: math.pi / 2}):
        after = ctx.part_world_position(turntable)
    ctx.check(
        "turntable yaws about centered vertical slew axis",
        before is not None and after is not None and abs(before[0]) < 1e-6 and abs(after[0]) < 1e-6 and abs(after[1]) < 1e-6,
        details=f"before={before}, after={after}",
    )

    return ctx.report()


object_model = build_object_model()
