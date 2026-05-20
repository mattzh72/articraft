from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    Mesh,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _cylinder_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    """Add a cylinder whose local +Z is aligned between two local points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0:
        raise ValueError("zero-length cylinder")
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _circle(cx: float, cy: float, r: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (cx + r * math.cos(2.0 * math.pi * i / segments), cy + r * math.sin(2.0 * math.pi * i / segments))
        for i in range(segments)
    ]


def _capsule_profile(
    p0: tuple[float, float],
    p1: tuple[float, float],
    radius: float,
    *,
    segments: int = 18,
) -> list[tuple[float, float]]:
    """2D rounded bar outline from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    angle = math.atan2(dy, dx)
    pts: list[tuple[float, float]] = []
    # Semicircle around the far end, then the start end.  Counter-clockwise.
    for i in range(segments + 1):
        theta = angle - math.pi / 2.0 + math.pi * i / segments
        pts.append((p1[0] + radius * math.cos(theta), p1[1] + radius * math.sin(theta)))
    for i in range(segments + 1):
        theta = angle + math.pi / 2.0 + math.pi * i / segments
        pts.append((p0[0] + radius * math.cos(theta), p0[1] + radius * math.sin(theta)))
    return pts


def _link_mesh(
    name: str,
    p0: tuple[float, float],
    p1: tuple[float, float],
    *,
    outer_radius: float,
    hole_radius: float,
    thickness: float,
) -> Mesh:
    """Flat stamped link with two through holes; cheap to stamp and easy to assemble."""
    geom = ExtrudeWithHolesGeometry(
        _capsule_profile(p0, p1, outer_radius),
        [_circle(p0[0], p0[1], hole_radius), _circle(p1[0], p1[1], hole_radius)],
        thickness,
        center=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="cost_optimized_windshield_wiper",
        meta={
            "design_notes": (
                "Cost-optimized production layout: one stamped rail carries two supported "
                "spindle pivots; a fixed motor/gearbox bolts to a shallow tray; flat stamped "
                "crank and links use snap-on ball studs; blade carriers are simple extruded "
                "spines with rubber inserts."
            )
        },
    )

    zinc = model.material("zinc_plated_steel", rgba=(0.58, 0.61, 0.62, 1.0))
    dark_steel = model.material("dark_phosphate_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    diecast = model.material("diecast_aluminum", rgba=(0.66, 0.68, 0.66, 1.0))
    black_plastic = model.material("black_glass_filled_plastic", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    yellow = model.material("yellow_service_clips", rgba=(0.95, 0.74, 0.12, 1.0))

    # Root: a shallow stamped rail with folded flanges and integral simple brackets.
    frame = model.part("frame")
    frame.visual(Box((1.18, 0.070, 0.026)), origin=Origin(xyz=(0.0, 0.0, 0.065)), material=zinc, name="stamped_rail")
    frame.visual(Box((1.18, 0.014, 0.052)), origin=Origin(xyz=(0.0, -0.042, 0.058)), material=zinc, name="rear_fold")
    frame.visual(Box((1.18, 0.012, 0.036)), origin=Origin(xyz=(0.0, 0.041, 0.067)), material=zinc, name="front_fold")
    frame.visual(Box((0.34, 0.150, 0.022)), origin=Origin(xyz=(0.0, -0.100, 0.060)), material=zinc, name="motor_tray_web")
    frame.visual(Box((0.34, 0.230, 0.022)), origin=Origin(xyz=(0.0, -0.210, 0.078)), material=zinc, name="motor_tray")
    frame.visual(Box((0.055, 0.210, 0.034)), origin=Origin(xyz=(-0.140, -0.160, 0.039)), material=zinc, name="tray_gusset_0")
    frame.visual(Box((0.055, 0.210, 0.034)), origin=Origin(xyz=(0.140, -0.160, 0.039)), material=zinc, name="tray_gusset_1")

    # Bolt pads and bolt heads make the assembly order obvious: rail first, motor next.
    for i, x in enumerate((-0.55, 0.55)):
        frame.visual(Box((0.120, 0.105, 0.018)), origin=Origin(xyz=(x, 0.0, 0.050)), material=zinc, name=f"end_mount_pad_{i}")
        frame.visual(Cylinder(radius=0.015, length=0.010), origin=Origin(xyz=(x, 0.0, 0.071)), material=dark_steel, name=f"end_bolt_{i}")
    for i, (x, y) in enumerate(((-0.120, -0.285), (0.120, -0.285), (-0.120, -0.140), (0.120, -0.140))):
        frame.visual(Cylinder(radius=0.012, length=0.010), origin=Origin(xyz=(x, y, 0.092)), material=dark_steel, name=f"motor_tray_bolt_{i}")

    # Two identical supported spindles with bushings welded/staked into the rail.
    pivot_x = (-0.420, 0.420)
    for i, x in enumerate(pivot_x):
        frame.visual(Cylinder(radius=0.052, length=0.020), origin=Origin(xyz=(x, 0.020, 0.086)), material=zinc, name=f"pedestal_foot_{i}")
        frame.visual(Cylinder(radius=0.035, length=0.068), origin=Origin(xyz=(x, 0.020, 0.109)), material=zinc, name=f"pedestal_tube_{i}")
        frame.visual(Cylinder(radius=0.026, length=0.082), origin=Origin(xyz=(x, 0.020, 0.117)), material=black_plastic, name=f"bushing_{i}")
        frame.visual(Box((0.022, 0.130, 0.072)), origin=Origin(xyz=(x - 0.042, -0.006, 0.092)), material=zinc, name=f"pedestal_rib_{i}_0")
        frame.visual(Box((0.022, 0.130, 0.072)), origin=Origin(xyz=(x + 0.042, -0.006, 0.092)), material=zinc, name=f"pedestal_rib_{i}_1")
        frame.visual(Cylinder(radius=0.015, length=0.012), origin=Origin(xyz=(x - 0.070, 0.010, 0.083)), material=dark_steel, name=f"pedestal_bolt_{i}_0")
        frame.visual(Cylinder(radius=0.015, length=0.012), origin=Origin(xyz=(x + 0.070, 0.010, 0.083)), material=dark_steel, name=f"pedestal_bolt_{i}_1")

    # Fixed motor/gearbox module: bolted from below into the tray, no separate molded brackets.
    motor_housing = model.part("motor_housing")
    motor_housing.visual(Cylinder(radius=0.108, length=0.040), origin=Origin(), material=diecast, name="gearbox_cover")
    motor_housing.visual(Cylinder(radius=0.055, length=0.235), origin=Origin(xyz=(0.220, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)), material=black_plastic, name="motor_can")
    motor_housing.visual(Cylinder(radius=0.062, length=0.020), origin=Origin(xyz=(0.105, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)), material=diecast, name="motor_can_crimp")
    motor_housing.visual(Cylinder(radius=0.031, length=0.060), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=dark_steel, name="output_bearing")
    motor_housing.visual(Box((0.250, 0.030, 0.020)), origin=Origin(xyz=(0.0, -0.120, -0.010)), material=diecast, name="lower_mount_lug")
    motor_housing.visual(Box((0.050, 0.105, 0.020)), origin=Origin(xyz=(-0.118, 0.020, -0.010)), material=diecast, name="side_mount_lug_0")
    motor_housing.visual(Box((0.050, 0.105, 0.020)), origin=Origin(xyz=(0.118, 0.020, -0.010)), material=diecast, name="side_mount_lug_1")
    motor_housing.visual(Box((0.050, 0.045, 0.032)), origin=Origin(xyz=(0.335, -0.010, 0.004)), material=black_plastic, name="wire_connector")
    motor_housing.visual(Box((0.018, 0.054, 0.010)), origin=Origin(xyz=(0.368, -0.010, 0.003)), material=yellow, name="connector_lock")

    model.articulation(
        "frame_to_motor_housing",
        ArticulationType.FIXED,
        parent=frame,
        child=motor_housing,
        origin=Origin(xyz=(0.0, -0.205, 0.108)),
    )

    # Motor output crank: flat stamped arm, one formed hub, one snap ball stud.
    motor_crank = model.part("motor_crank")
    motor_crank.visual(Cylinder(radius=0.014, length=0.070), origin=Origin(xyz=(0.0, 0.0, -0.010)), material=dark_steel, name="output_shaft")
    motor_crank.visual(Cylinder(radius=0.030, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=zinc, name="crank_hub")
    motor_crank.visual(
        _link_mesh("crank_arm_plate", (0.0, 0.0), (-0.110, 0.020), outer_radius=0.025, hole_radius=0.013, thickness=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=zinc,
        name="crank_arm",
    )
    motor_crank.visual(Cylinder(radius=0.009, length=0.034), origin=Origin(xyz=(-0.110, 0.020, 0.020)), material=dark_steel, name="crank_pin")
    motor_crank.visual(Sphere(radius=0.014), origin=Origin(xyz=(-0.110, 0.020, 0.040)), material=dark_steel, name="crank_ball")
    motor_crank.visual(Cylinder(radius=0.017, length=0.005), origin=Origin(xyz=(-0.110, 0.020, 0.056)), material=yellow, name="crank_snap_clip")

    model.articulation(
        "motor_to_crank",
        ArticulationType.REVOLUTE,
        parent=motor_housing,
        child=motor_crank,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=9.0, velocity=4.0, lower=-1.10, upper=1.10),
    )

    # One drive link from the crank pin to the first spindle rocker.
    drive_link = model.part("drive_link")
    drive_link.visual(
        _link_mesh("drive_link_plate", (0.0, 0.0), (-0.260, 0.060), outer_radius=0.022, hole_radius=0.0125, thickness=0.0055),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=zinc,
        name="drive_link_plate",
    )
    drive_link.visual(Box((0.026, 0.010, 0.010)), origin=Origin(xyz=(-0.024, -0.019, 0.010)), material=yellow, name="link_snap_tab_0")
    drive_link.visual(Box((0.026, 0.010, 0.010)), origin=Origin(xyz=(-0.236, 0.080, 0.010)), material=yellow, name="link_snap_tab_1")
    model.articulation(
        "crank_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(-0.110, 0.020, 0.020)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=-0.45, upper=0.45),
    )

    def add_spindle(index: int, x: float, arm_tip: tuple[float, float, float], cross_sign: float) -> None:
        spindle = model.part(f"spindle_{index}")
        # Shaft is intentionally captured by the fixed bushing; allow this scoped overlap in tests.
        spindle.visual(Cylinder(radius=0.012, length=0.130), origin=Origin(xyz=(0.0, 0.0, 0.017)), material=dark_steel, name="spindle_shaft")
        spindle.visual(Cylinder(radius=0.026, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.086)), material=zinc, name="splined_hub")
        spindle.visual(Cylinder(radius=0.032, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.108)), material=black_plastic, name="weather_cap")
        spindle.visual(
            _link_mesh(f"rocker_arm_{index}", (0.0, 0.0), (0.050 * cross_sign, -0.120), outer_radius=0.026, hole_radius=0.014, thickness=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=zinc,
            name="drive_rocker",
        )
        spindle.visual(
            _link_mesh(f"cross_rocker_{index}", (0.0, 0.0), (0.100 * cross_sign, -0.075), outer_radius=0.026, hole_radius=0.014, thickness=0.007),
            origin=Origin(xyz=(0.0, 0.0, 0.023)),
            material=zinc,
            name="cross_rocker",
        )
        spindle.visual(Cylinder(radius=0.009, length=0.034), origin=Origin(xyz=(0.050 * cross_sign, -0.120, 0.023)), material=dark_steel, name="drive_ball_pin")
        spindle.visual(Sphere(radius=0.013), origin=Origin(xyz=(0.050 * cross_sign, -0.120, 0.043)), material=dark_steel, name="drive_ball")
        spindle.visual(Cylinder(radius=0.009, length=0.030), origin=Origin(xyz=(0.100 * cross_sign, -0.075, 0.023)), material=dark_steel, name="cross_pin")

        # Visible blade arm: a low-cost roll-formed/stamped U-channel with a riveted tip saddle.
        _cylinder_between(spindle, (0.0, 0.0, 0.095), arm_tip, radius=0.011, material=dark_steel, name="arm_channel")
        _cylinder_between(spindle, (0.0, 0.0, 0.099), (arm_tip[0] * 0.88, arm_tip[1] * 0.88, arm_tip[2]), radius=0.004, material=zinc, name="pressed_arm_rib")
        spindle.visual(Box((0.070, 0.026, 0.020)), origin=Origin(xyz=(arm_tip[0], arm_tip[1], arm_tip[2])), material=black_plastic, name="tip_clevis")
        spindle.visual(Cylinder(radius=0.0065, length=0.082), origin=Origin(xyz=(arm_tip[0], arm_tip[1], arm_tip[2]), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_steel, name="tip_pin")
        spindle.visual(Box((0.050, 0.020, 0.012)), origin=Origin(xyz=(0.0, 0.004, 0.112)), material=yellow, name="service_release")

        model.articulation(
            f"frame_to_spindle_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=spindle,
            origin=Origin(xyz=(x, 0.020, 0.145)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=7.0, velocity=2.0, lower=-0.62, upper=0.62),
            mimic=Mimic(joint="motor_to_crank", multiplier=0.56, offset=0.0),
        )

        blade = model.part(f"blade_{index}")
        blade.visual(Box((0.455, 0.030, 0.012)), origin=Origin(xyz=(0.0, 0.0, -0.045)), material=black_plastic, name="blade_spine")
        blade.visual(Box((0.430, 0.012, 0.030)), origin=Origin(xyz=(0.0, -0.004, -0.060)), material=rubber, name="rubber_insert")
        blade.visual(Box((0.070, 0.044, 0.070)), origin=Origin(xyz=(0.0, 0.0, -0.020)), material=black_plastic, name="center_saddle")
        blade.visual(Box((0.030, 0.036, 0.014)), origin=Origin(xyz=(-0.210, 0.0, -0.045)), material=black_plastic, name="end_cap_0")
        blade.visual(Box((0.030, 0.036, 0.014)), origin=Origin(xyz=(0.210, 0.0, -0.045)), material=black_plastic, name="end_cap_1")

        model.articulation(
            f"spindle_{index}_to_blade",
            ArticulationType.REVOLUTE,
            parent=spindle,
            child=blade,
            origin=Origin(xyz=arm_tip),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=1.0, lower=-0.20, upper=0.20),
        )

    add_spindle(0, -0.420, (0.180, 0.565, 0.125), 1.0)
    add_spindle(1, 0.420, (-0.180, 0.565, 0.125), -1.0)

    # Cross link is assembled last onto the two exposed rocker pins with yellow snap retainers.
    cross_link = model.part("cross_link")
    cross_link.visual(
        _link_mesh("cross_link_plate", (0.0, 0.0), (0.640, 0.0), outer_radius=0.020, hole_radius=0.0125, thickness=0.0055),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=zinc,
        name="cross_link_plate",
    )
    cross_link.visual(Box((0.024, 0.010, 0.010)), origin=Origin(xyz=(0.030, -0.018, 0.004)), material=yellow, name="cross_snap_tab_0")
    cross_link.visual(Box((0.024, 0.010, 0.010)), origin=Origin(xyz=(0.610, -0.018, 0.004)), material=yellow, name="cross_snap_tab_1")
    cross_link.visual(Cylinder(radius=0.0135, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=black_plastic, name="cross_bushing_0")
    cross_link.visual(Cylinder(radius=0.0135, length=0.010), origin=Origin(xyz=(0.640, 0.0, 0.0)), material=black_plastic, name="cross_bushing_1")
    model.articulation(
        "spindle_0_to_cross_link",
        ArticulationType.REVOLUTE,
        parent="spindle_0",
        child=cross_link,
        origin=Origin(xyz=(0.100, -0.075, 0.023)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=-0.30, upper=0.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    motor_housing = object_model.get_part("motor_housing")
    motor_crank = object_model.get_part("motor_crank")
    spindle_0 = object_model.get_part("spindle_0")
    spindle_1 = object_model.get_part("spindle_1")
    blade_0 = object_model.get_part("blade_0")
    blade_1 = object_model.get_part("blade_1")
    motor_joint = object_model.get_articulation("motor_to_crank")

    # Intentional captured fits.  These are local, named interfaces: the model
    # uses simplified solid bushings and saddles instead of full bearing holes.
    ctx.allow_overlap(
        motor_housing,
        motor_crank,
        elem_a="output_bearing",
        elem_b="output_shaft",
        reason="The crank output shaft is intentionally captured inside the gearbox bearing proxy.",
    )
    ctx.expect_within(
        motor_crank,
        motor_housing,
        axes="xy",
        inner_elem="output_shaft",
        outer_elem="output_bearing",
        margin=0.001,
        name="motor shaft centered in bearing",
    )
    ctx.expect_overlap(
        motor_crank,
        motor_housing,
        axes="z",
        elem_a="output_shaft",
        elem_b="output_bearing",
        min_overlap=0.030,
        name="motor shaft retained by bearing length",
    )
    ctx.allow_overlap(
        motor_housing,
        motor_crank,
        elem_a="gearbox_cover",
        elem_b="output_shaft",
        reason="The shaft passes through the simplified solid gearbox cover before the exposed crank hub.",
    )
    ctx.expect_overlap(
        motor_crank,
        motor_housing,
        axes="z",
        elem_a="output_shaft",
        elem_b="gearbox_cover",
        min_overlap=0.018,
        name="shaft passes through gearbox cover",
    )

    for index, spindle in enumerate((spindle_0, spindle_1)):
        ctx.allow_overlap(
            frame,
            spindle,
            elem_a=f"bushing_{index}",
            elem_b="spindle_shaft",
            reason="The rotating spindle shaft is intentionally supported inside the fixed bushing proxy.",
        )
        ctx.allow_overlap(
            frame,
            spindle,
            elem_a=f"pedestal_tube_{index}",
            elem_b="spindle_shaft",
            reason="The pedestal is a simplified solid sleeve around the same supported spindle shaft.",
        )
        ctx.expect_within(
            spindle,
            frame,
            axes="xy",
            inner_elem="spindle_shaft",
            outer_elem=f"bushing_{index}",
            margin=0.001,
            name=f"spindle {index} shaft centered in bushing",
        )
        ctx.expect_overlap(
            spindle,
            frame,
            axes="z",
            elem_a="spindle_shaft",
            elem_b=f"bushing_{index}",
            min_overlap=0.055,
            name=f"spindle {index} has long supported bearing engagement",
        )

    for lug_name, bolt_name in (("side_mount_lug_0", "motor_tray_bolt_2"), ("side_mount_lug_1", "motor_tray_bolt_3")):
        ctx.allow_overlap(
            frame,
            motor_housing,
            elem_a=bolt_name,
            elem_b=lug_name,
            reason="Tray bolt heads are intentionally represented as passing through the motor mounting lugs.",
        )
        ctx.expect_overlap(
            frame,
            motor_housing,
            axes="xy",
            elem_a=bolt_name,
            elem_b=lug_name,
            min_overlap=0.018,
            name=f"{lug_name} is captured by its tray bolt",
        )

    for index, (spindle, blade) in enumerate(((spindle_0, blade_0), (spindle_1, blade_1))):
        ctx.allow_overlap(
            spindle,
            blade,
            elem_a="tip_pin",
            elem_b="center_saddle",
            reason="The blade carrier saddle is represented as a solid clevis captured by the rivet pin.",
        )
        ctx.expect_overlap(
            spindle,
            blade,
            axes="x",
            elem_a="tip_pin",
            elem_b="center_saddle",
            min_overlap=0.055,
            name=f"blade {index} saddle spans its rivet pin",
        )
        ctx.expect_within(
            blade,
            spindle,
            axes="xy",
            inner_elem="center_saddle",
            outer_elem="tip_clevis",
            margin=0.020,
            name=f"blade {index} saddle sits inside tip clevis footprint",
        )
        ctx.allow_overlap(
            spindle,
            blade,
            elem_a="tip_clevis",
            elem_b="center_saddle",
            reason="The molded blade saddle is represented as seated inside the simplified solid tip clevis.",
        )
        ctx.allow_overlap(
            spindle,
            blade,
            elem_a="arm_channel",
            elem_b="center_saddle",
            reason="The roll-formed arm end is intentionally seated into the simplified solid blade saddle.",
        )
        ctx.expect_overlap(
            spindle,
            blade,
            axes="xy",
            elem_a="tip_clevis",
            elem_b="center_saddle",
            min_overlap=0.020,
            name=f"blade {index} saddle captured by tip clevis",
        )

    drive_link = object_model.get_part("drive_link")
    cross_link = object_model.get_part("cross_link")
    ctx.allow_overlap(
        motor_crank,
        drive_link,
        elem_a="crank_pin",
        elem_b="drive_link_plate",
        reason="The crank pin is captured through the stamped drive-link eye with a snap retainer.",
    )
    ctx.expect_overlap(
        motor_crank,
        drive_link,
        axes="z",
        elem_a="crank_pin",
        elem_b="drive_link_plate",
        min_overlap=0.004,
        name="drive link retained on crank pin",
    )
    ctx.allow_overlap(
        spindle_0,
        drive_link,
        elem_a="drive_ball",
        elem_b="drive_link_plate",
        reason="The serviceable plastic-lined drive-link eye snaps over the first spindle ball stud.",
    )
    ctx.allow_overlap(
        spindle_0,
        drive_link,
        elem_a="drive_ball_pin",
        elem_b="drive_link_plate",
        reason="The drive-link eye is captured on the same pressed ball-stud shank.",
    )
    ctx.expect_overlap(
        spindle_0,
        drive_link,
        axes="z",
        elem_a="drive_ball",
        elem_b="drive_link_plate",
        min_overlap=0.004,
        name="drive link retained on spindle ball",
    )
    ctx.expect_overlap(
        spindle_0,
        drive_link,
        axes="z",
        elem_a="drive_ball_pin",
        elem_b="drive_link_plate",
        min_overlap=0.004,
        name="drive link retained on spindle ball shank",
    )
    ctx.allow_overlap(
        spindle_0,
        cross_link,
        elem_a="cross_pin",
        elem_b="cross_bushing_0",
        reason="The cross-link bushing snaps over the first spindle rocker pin.",
    )
    ctx.allow_overlap(
        spindle_0,
        cross_link,
        elem_a="cross_pin",
        elem_b="cross_link_plate",
        reason="The first rocker pin passes through the simplified stamped cross-link eye.",
    )
    ctx.allow_overlap(
        spindle_0,
        cross_link,
        elem_a="cross_rocker",
        elem_b="cross_bushing_0",
        reason="The cross-link bushing seats against the simplified flat rocker eye.",
    )
    ctx.allow_overlap(
        spindle_0,
        cross_link,
        elem_a="cross_rocker",
        elem_b="cross_link_plate",
        reason="The stamped cross-link eye is seated directly on the first rocker plate in the simplified stack.",
    )
    ctx.allow_overlap(
        spindle_1,
        cross_link,
        elem_a="cross_pin",
        elem_b="cross_bushing_1",
        reason="The cross-link bushing snaps over the second spindle rocker pin.",
    )
    ctx.allow_overlap(
        spindle_1,
        cross_link,
        elem_a="cross_pin",
        elem_b="cross_link_plate",
        reason="The second rocker pin passes through the simplified stamped cross-link eye.",
    )
    ctx.allow_overlap(
        spindle_1,
        cross_link,
        elem_a="cross_rocker",
        elem_b="cross_bushing_1",
        reason="The cross-link bushing seats against the simplified flat rocker eye.",
    )
    ctx.allow_overlap(
        spindle_1,
        cross_link,
        elem_a="cross_rocker",
        elem_b="cross_link_plate",
        reason="The stamped cross-link eye is seated directly on the second rocker plate in the simplified stack.",
    )
    ctx.expect_overlap(
        spindle_0,
        cross_link,
        axes="z",
        elem_a="cross_pin",
        elem_b="cross_bushing_0",
        min_overlap=0.006,
        name="cross link retained on first spindle pin",
    )
    ctx.expect_overlap(
        spindle_1,
        cross_link,
        axes="z",
        elem_a="cross_pin",
        elem_b="cross_bushing_1",
        min_overlap=0.006,
        name="cross link retained on second spindle pin",
    )

    def _center_x(aabb):
        return (aabb[0][0] + aabb[1][0]) * 0.5 if aabb is not None else None

    with ctx.pose({motor_joint: 0.0}):
        crank_x0 = _center_x(ctx.part_element_world_aabb(motor_crank, elem="crank_pin"))
        arm_x0 = _center_x(ctx.part_element_world_aabb(spindle_0, elem="tip_clevis"))
        blade_x0 = _center_x(ctx.part_element_world_aabb(blade_0, elem="rubber_insert"))

    with ctx.pose({motor_joint: 0.85}):
        crank_x1 = _center_x(ctx.part_element_world_aabb(motor_crank, elem="crank_pin"))
        arm_x1 = _center_x(ctx.part_element_world_aabb(spindle_0, elem="tip_clevis"))
        blade_x1 = _center_x(ctx.part_element_world_aabb(blade_0, elem="rubber_insert"))

    ctx.check(
        "crank pin sweeps from motor drive",
        crank_x0 is not None and crank_x1 is not None and abs(crank_x1 - crank_x0) > 0.020,
        details=f"crank_x0={crank_x0}, crank_x1={crank_x1}",
    )
    ctx.check(
        "spindle arm follows constrained sweep",
        arm_x0 is not None and arm_x1 is not None and abs(arm_x1 - arm_x0) > 0.045,
        details=f"arm_x0={arm_x0}, arm_x1={arm_x1}",
    )
    ctx.check(
        "blade carrier moves with spindle",
        blade_x0 is not None and blade_x1 is not None and abs(blade_x1 - blade_x0) > 0.045,
        details=f"blade_x0={blade_x0}, blade_x1={blade_x1}",
    )

    return ctx.report()


object_model = build_object_model()
