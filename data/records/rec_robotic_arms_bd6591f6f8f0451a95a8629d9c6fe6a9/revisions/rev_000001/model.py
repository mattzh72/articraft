from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_robot_arm")

    galvanized = model.material("hot_dip_galvanized_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    painted = model.material("marine_blue_powder_coat", rgba=(0.08, 0.22, 0.34, 1.0))
    dark_paint = model.material("charcoal_epoxy_paint", rgba=(0.06, 0.07, 0.075, 1.0))
    rubber = model.material("black_epdm_seal", rgba=(0.004, 0.004, 0.004, 1.0))
    stainless = model.material("brushed_stainless_hardware", rgba=(0.82, 0.82, 0.78, 1.0))
    warning = model.material("yellow_service_index_marks", rgba=(0.95, 0.72, 0.08, 1.0))

    cyl_y = (pi / 2.0, 0.0, 0.0)
    cyl_x = (0.0, pi / 2.0, 0.0)

    def add_radial_bolts(
        part,
        *,
        prefix: str,
        center_x: float = 0.0,
        center_y: float = 0.0,
        center_z: float = 0.0,
        radius: float,
        bolt_radius: float,
        bolt_length: float,
        material: Material,
        axis: str = "z",
        count: int = 8,
    ) -> None:
        """Small cap screws seated slightly into a flange or cartridge face."""
        for index in range(count):
            angle = (2.0 * pi * index) / count
            if axis == "z":
                xyz = (
                    center_x + radius * cos(angle),
                    center_y + radius * sin(angle),
                    center_z,
                )
                rpy = (0.0, 0.0, 0.0)
            elif axis == "y":
                xyz = (
                    center_x + radius * cos(angle),
                    center_y,
                    center_z + radius * sin(angle),
                )
                rpy = cyl_y
            else:
                xyz = (
                    center_x,
                    center_y + radius * cos(angle),
                    center_z + radius * sin(angle),
                )
                rpy = cyl_x
            part.visual(
                Cylinder(radius=bolt_radius, length=bolt_length),
                origin=Origin(xyz=xyz, rpy=rpy),
                material=material,
                name=f"{prefix}_bolt_{index}",
            )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.50, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=galvanized,
        name="concrete_anchor_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.43, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=painted,
        name="base_flange",
    )
    pedestal.visual(
        Cylinder(radius=0.18, length=0.485),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=painted,
        name="sealed_pedestal_tube",
    )
    pedestal.visual(
        Cylinder(radius=0.235, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.595)),
        material=dark_paint,
        name="drip_skirt",
    )
    pedestal.visual(
        Cylinder(radius=0.265, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        material=painted,
        name="top_rain_cap",
    )
    pedestal.visual(
        Box((0.060, 0.110, 0.355)),
        origin=Origin(xyz=(-0.182, 0.0, 0.355)),
        material=dark_paint,
        name="rear_cable_chase",
    )
    pedestal.visual(
        Box((0.090, 0.150, 0.060)),
        origin=Origin(xyz=(-0.230, 0.0, 0.555)),
        material=rubber,
        name="sealed_cable_gland",
    )
    add_radial_bolts(
        pedestal,
        prefix="anchor",
        center_z=0.145,
        radius=0.345,
        bolt_radius=0.018,
        bolt_length=0.018,
        material=stainless,
        count=10,
    )
    add_radial_bolts(
        pedestal,
        prefix="cap",
        center_z=0.644,
        radius=0.205,
        bolt_radius=0.010,
        bolt_length=0.012,
        material=stainless,
        count=8,
    )

    shoulder = model.part("shoulder")
    shoulder.visual(
        Cylinder(radius=0.245, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=rubber,
        name="yaw_lower_seal",
    )
    shoulder.visual(
        Cylinder(radius=0.220, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=painted,
        name="yaw_cartridge_drum",
    )
    shoulder.visual(
        Cylinder(radius=0.275, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=painted,
        name="yaw_rain_hat",
    )
    shoulder.visual(
        Box((0.500, 0.340, 0.105)),
        origin=Origin(xyz=(0.120, 0.0, 0.247)),
        material=painted,
        name="shoulder_yoke_base",
    )
    for side, y in (("side_0", -0.130), ("side_1", 0.130)):
        shoulder.visual(
            Box((0.180, 0.060, 0.390)),
            origin=Origin(xyz=(0.120, y, 0.470)),
            material=painted,
            name="shoulder_cheek_side_0" if side == "side_0" else "shoulder_cheek_side_1",
        )
        shoulder.visual(
            Cylinder(radius=0.115, length=0.038),
            origin=Origin(xyz=(0.120, y * 1.365, 0.470), rpy=cyl_y),
            material=dark_paint,
            name=f"shoulder_outer_cap_{side}",
        )
    shoulder.visual(
        Box((0.230, 0.390, 0.040)),
        origin=Origin(xyz=(0.120, 0.0, 0.682)),
        material=painted,
        name="shoulder_drip_brow",
    )
    add_radial_bolts(
        shoulder,
        prefix="shoulder_cap_0",
        center_x=0.120,
        center_y=-0.182,
        center_z=0.470,
        radius=0.082,
        bolt_radius=0.0065,
        bolt_length=0.010,
        material=stainless,
        axis="y",
        count=6,
    )
    add_radial_bolts(
        shoulder,
        prefix="shoulder_cap_1",
        center_x=0.120,
        center_y=0.182,
        center_z=0.470,
        radius=0.082,
        bolt_radius=0.0065,
        bolt_length=0.010,
        material=stainless,
        axis="y",
        count=6,
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.105, length=0.200),
        origin=Origin(rpy=cyl_y),
        material=dark_paint,
        name="shoulder_hub",
    )
    for side, y in (("side_0", -0.093), ("side_1", 0.093)):
        upper_arm.visual(
            Cylinder(radius=0.116, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_y),
            material=rubber,
            name="shoulder_face_seal_side_0" if side == "side_0" else "shoulder_face_seal_side_1",
        )
    upper_arm.visual(
        Box((0.620, 0.120, 0.140)),
        origin=Origin(xyz=(0.410, 0.0, 0.0)),
        material=painted,
        name="upper_sealed_box_beam",
    )
    upper_arm.visual(
        Box((0.620, 0.170, 0.035)),
        origin=Origin(xyz=(0.410, 0.0, 0.085)),
        material=painted,
        name="upper_drip_cover",
    )
    upper_arm.visual(
        Box((0.590, 0.060, 0.050)),
        origin=Origin(xyz=(0.425, 0.0, -0.085)),
        material=dark_paint,
        name="upper_protected_cable_duct",
    )
    for side, y in (("side_0", -0.130), ("side_1", 0.130)):
        upper_arm.visual(
            Box((0.165, 0.060, 0.300)),
            origin=Origin(xyz=(0.840, y, 0.0)),
            material=painted,
            name="elbow_cheek_side_0" if side == "side_0" else "elbow_cheek_side_1",
        )
        upper_arm.visual(
            Cylinder(radius=0.095, length=0.034),
            origin=Origin(xyz=(0.840, y * 1.3615, 0.0), rpy=cyl_y),
            material=dark_paint,
            name=f"elbow_outer_cap_{side}",
        )
    upper_arm.visual(
        Box((0.230, 0.090, 0.052)),
        origin=Origin(xyz=(0.735, 0.0, 0.124)),
        material=painted,
        name="elbow_upper_bridge",
    )
    upper_arm.visual(
        Box((0.205, 0.375, 0.036)),
        origin=Origin(xyz=(0.840, 0.0, 0.164)),
        material=painted,
        name="elbow_drip_brow",
    )
    add_radial_bolts(
        upper_arm,
        prefix="elbow_cap_0",
        center_x=0.840,
        center_y=-0.178,
        center_z=0.0,
        radius=0.066,
        bolt_radius=0.0055,
        bolt_length=0.009,
        material=stainless,
        axis="y",
        count=6,
    )
    add_radial_bolts(
        upper_arm,
        prefix="elbow_cap_1",
        center_x=0.840,
        center_y=0.178,
        center_z=0.0,
        radius=0.066,
        bolt_radius=0.0055,
        bolt_length=0.009,
        material=stainless,
        axis="y",
        count=6,
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.094, length=0.200),
        origin=Origin(rpy=cyl_y),
        material=dark_paint,
        name="elbow_hub",
    )
    for side, y in (("side_0", -0.093), ("side_1", 0.093)):
        forearm.visual(
            Cylinder(radius=0.104, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_y),
            material=rubber,
            name="elbow_face_seal_side_0" if side == "side_0" else "elbow_face_seal_side_1",
        )
    forearm.visual(
        Box((0.512, 0.105, 0.120)),
        origin=Origin(xyz=(0.344, 0.0, 0.0)),
        material=painted,
        name="forearm_sealed_box_beam",
    )
    forearm.visual(
        Box((0.510, 0.148, 0.032)),
        origin=Origin(xyz=(0.350, 0.0, 0.074)),
        material=painted,
        name="forearm_drip_cover",
    )
    forearm.visual(
        Box((0.450, 0.050, 0.042)),
        origin=Origin(xyz=(0.360, 0.0, -0.071)),
        material=dark_paint,
        name="forearm_cable_duct",
    )
    for side, y in (("side_0", -0.105), ("side_1", 0.105)):
        forearm.visual(
            Box((0.125, 0.050, 0.230)),
            origin=Origin(xyz=(0.680, y, 0.0)),
            material=painted,
            name="wrist_pitch_cheek_side_0" if side == "side_0" else "wrist_pitch_cheek_side_1",
        )
        forearm.visual(
            Cylinder(radius=0.074, length=0.028),
            origin=Origin(xyz=(0.680, y * 1.350, 0.0), rpy=cyl_y),
            material=dark_paint,
            name=f"wrist_pitch_cap_{side}",
        )
    forearm.visual(
        Box((0.170, 0.078, 0.034)),
        origin=Origin(xyz=(0.610, 0.0, 0.095)),
        material=painted,
        name="wrist_upper_bridge",
    )
    forearm.visual(
        Box((0.160, 0.300, 0.032)),
        origin=Origin(xyz=(0.680, 0.0, 0.126)),
        material=painted,
        name="wrist_pitch_drip_brow",
    )

    wrist_pitch = model.part("wrist_pitch")
    wrist_pitch.visual(
        Cylinder(radius=0.074, length=0.160),
        origin=Origin(rpy=cyl_y),
        material=dark_paint,
        name="wrist_pitch_hub",
    )
    for side, y in (("side_0", -0.073), ("side_1", 0.073)):
        wrist_pitch.visual(
            Cylinder(radius=0.083, length=0.014),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=cyl_y),
            material=rubber,
            name="wrist_pitch_face_seal_side_0" if side == "side_0" else "wrist_pitch_face_seal_side_1",
        )
    wrist_pitch.visual(
        Box((0.210, 0.095, 0.095)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=painted,
        name="wrist_sealed_cradle",
    )
    wrist_pitch.visual(
        Box((0.220, 0.130, 0.028)),
        origin=Origin(xyz=(0.150, 0.0, 0.061)),
        material=painted,
        name="wrist_drip_lid",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.090, length=0.100),
        origin=Origin(xyz=(0.220, 0.0, 0.0), rpy=cyl_x),
        material=dark_paint,
        name="roll_socket_cartridge",
    )
    wrist_pitch.visual(
        Cylinder(radius=0.100, length=0.022),
        origin=Origin(xyz=(0.259, 0.0, 0.0), rpy=cyl_x),
        material=rubber,
        name="roll_face_seal",
    )
    add_radial_bolts(
        wrist_pitch,
        prefix="roll_socket",
        center_x=0.266,
        center_y=0.0,
        center_z=0.0,
        radius=0.068,
        bolt_radius=0.0048,
        bolt_length=0.008,
        material=stainless,
        axis="x",
        count=8,
    )

    wrist_tool = model.part("wrist_tool")
    wrist_tool.visual(
        Cylinder(radius=0.070, length=0.105),
        origin=Origin(xyz=(0.0525, 0.0, 0.0), rpy=cyl_x),
        material=dark_paint,
        name="roll_output_shaft",
    )
    wrist_tool.visual(
        Cylinder(radius=0.160, length=0.038),
        origin=Origin(xyz=(0.123, 0.0, 0.0), rpy=cyl_x),
        material=painted,
        name="tool_mount_flange",
    )
    wrist_tool.visual(
        Cylinder(radius=0.115, length=0.014),
        origin=Origin(xyz=(0.149, 0.0, 0.0), rpy=cyl_x),
        material=stainless,
        name="stainless_tool_face",
    )
    wrist_tool.visual(
        Box((0.028, 0.070, 0.012)),
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
        material=warning,
        name="roll_axis_index",
    )
    add_radial_bolts(
        wrist_tool,
        prefix="tool_face",
        center_x=0.158,
        center_y=0.0,
        center_z=0.0,
        radius=0.102,
        bolt_radius=0.006,
        bolt_length=0.010,
        material=stainless,
        axis="x",
        count=8,
    )

    model.articulation(
        "pedestal_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, 0.650)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.8, lower=-2.80, upper=2.80),
    )
    model.articulation(
        "shoulder_to_upper",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=upper_arm,
        origin=Origin(xyz=(0.120, 0.0, 0.470)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=760.0, velocity=0.7, lower=-0.65, upper=1.25),
    )
    model.articulation(
        "upper_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.840, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=520.0, velocity=0.9, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "forearm_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist_pitch,
        origin=Origin(xyz=(0.680, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=170.0, velocity=1.2, lower=-1.40, upper=1.40),
    )
    model.articulation(
        "wrist_to_tool",
        ArticulationType.REVOLUTE,
        parent=wrist_pitch,
        child=wrist_tool,
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.8, lower=-pi, upper=pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    shoulder = object_model.get_part("shoulder")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist_pitch = object_model.get_part("wrist_pitch")
    wrist_tool = object_model.get_part("wrist_tool")

    yaw = object_model.get_articulation("pedestal_to_shoulder")
    shoulder_pitch = object_model.get_articulation("shoulder_to_upper")
    elbow_pitch = object_model.get_articulation("upper_to_forearm")
    wrist_bend = object_model.get_articulation("forearm_to_wrist")
    wrist_roll = object_model.get_articulation("wrist_to_tool")

    ctx.check(
        "single pedestal anchored chain",
        [part.name for part in object_model.root_parts()] == ["pedestal"]
        and len(object_model.parts) == 6
        and len(object_model.articulations) == 5,
        details="Expected one fixed pedestal feeding shoulder, upper arm, forearm, wrist, and tool flange.",
    )
    ctx.check(
        "readable axis order",
        yaw.axis == (0.0, 0.0, 1.0)
        and shoulder_pitch.axis == (0.0, -1.0, 0.0)
        and elbow_pitch.axis == (0.0, -1.0, 0.0)
        and wrist_bend.axis == (0.0, -1.0, 0.0)
        and wrist_roll.axis == (1.0, 0.0, 0.0),
        details="Expected Z yaw, then offset pitch-pitch-pitch, then X roll at the tool cartridge.",
    )

    ctx.expect_contact(
        shoulder,
        pedestal,
        elem_a="yaw_lower_seal",
        elem_b="top_rain_cap",
        contact_tol=0.001,
        name="yaw seal is seated on pedestal cap",
    )
    ctx.expect_contact(
        upper_arm,
        shoulder,
        elem_a="shoulder_face_seal_side_1",
        elem_b="shoulder_cheek_side_1",
        contact_tol=0.001,
        name="shoulder cartridge captured by yoke",
    )
    ctx.expect_contact(
        forearm,
        upper_arm,
        elem_a="elbow_face_seal_side_1",
        elem_b="elbow_cheek_side_1",
        contact_tol=0.001,
        name="elbow cartridge captured by yoke",
    )
    ctx.expect_contact(
        wrist_pitch,
        forearm,
        elem_a="wrist_pitch_face_seal_side_1",
        elem_b="wrist_pitch_cheek_side_1",
        contact_tol=0.001,
        name="wrist pitch cartridge captured by yoke",
    )
    ctx.expect_contact(
        wrist_tool,
        wrist_pitch,
        elem_a="roll_output_shaft",
        elem_b="roll_face_seal",
        contact_tol=0.001,
        name="wrist roll cartridge seated in seal",
    )

    rest_forearm_pos = ctx.part_world_position(forearm)
    rest_tool_pos = ctx.part_world_position(wrist_tool)
    with ctx.pose({shoulder_pitch: 0.55, elbow_pitch: 0.35, wrist_bend: -0.25, wrist_roll: 0.80}):
        raised_forearm_pos = ctx.part_world_position(forearm)
        moved_tool_pos = ctx.part_world_position(wrist_tool)
    ctx.check(
        "positive shoulder pitch raises downstream chain",
        rest_forearm_pos is not None
        and raised_forearm_pos is not None
        and raised_forearm_pos[2] > rest_forearm_pos[2] + 0.10,
        details=f"rest={rest_forearm_pos}, raised={raised_forearm_pos}",
    )
    ctx.check(
        "wrist remains downstream of elbow after compound pose",
        rest_tool_pos is not None
        and moved_tool_pos is not None
        and moved_tool_pos[0] > 0.80
        and moved_tool_pos[2] > 1.0,
        details=f"rest={rest_tool_pos}, moved={moved_tool_pos}",
    )

    return ctx.report()


object_model = build_object_model()
