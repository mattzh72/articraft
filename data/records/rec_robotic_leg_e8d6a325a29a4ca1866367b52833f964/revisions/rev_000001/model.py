from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_robotic_leg")

    armor = model.material("oiled_ochre_armor", rgba=(0.74, 0.57, 0.20, 1.0))
    dark = model.material("matte_graphite", rgba=(0.06, 0.065, 0.07, 1.0))
    steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.64, 1.0))
    rubber = model.material("worn_black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    service = model.material("blue_service_covers", rgba=(0.08, 0.18, 0.30, 1.0))

    def cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
        return Cylinder(radius=radius, length=length), Origin(rpy=(pi / 2.0, 0.0, 0.0))

    def cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
        return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))

    def add_bolt_x(part, name: str, xyz: tuple[float, float, float], radius: float = 0.012) -> None:
        geom, orient = cyl_x(radius, 0.014)
        part.visual(
            geom,
            origin=Origin(xyz=xyz, rpy=orient.rpy),
            material=steel,
            name=name,
        )

    def add_bolt_z(part, name: str, xyz: tuple[float, float, float], radius: float = 0.013) -> None:
        part.visual(
            Cylinder(radius=radius, length=0.014),
            origin=Origin(xyz=xyz),
            material=steel,
            name=name,
        )

    # Root hip bracket: a serviceable welded yoke with a through shaft and
    # replaceable bearing caps.  The root frame sits on the hip pitch axis.
    hip = model.part("hip_mount")
    hip.visual(Box((0.36, 0.56, 0.08)), origin=Origin(xyz=(0.0, 0.0, 0.18)), material=dark, name="top_plate")
    hip.visual(Box((0.26, 0.06, 0.30)), origin=Origin(xyz=(0.0, 0.22, 0.0)), material=dark, name="yoke_cheek_0")
    hip.visual(Box((0.26, 0.06, 0.30)), origin=Origin(xyz=(0.0, -0.22, 0.0)), material=dark, name="yoke_cheek_1")
    hip.visual(Box((0.11, 0.48, 0.18)), origin=Origin(xyz=(-0.155, 0.0, 0.055)), material=dark, name="rear_weldment")
    geom, orient = cyl_y(0.045, 0.58)
    hip.visual(geom, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=orient.rpy), material=steel, name="hip_pin")
    geom, orient = cyl_y(0.075, 0.045)
    hip.visual(geom, origin=Origin(xyz=(0.0, 0.305, 0.0), rpy=orient.rpy), material=steel, name="bearing_cap_0")
    hip.visual(geom, origin=Origin(xyz=(0.0, -0.305, 0.0), rpy=orient.rpy), material=steel, name="bearing_cap_1")
    hip.visual(Box((0.18, 0.30, 0.025)), origin=Origin(xyz=(0.055, 0.0, 0.232)), material=service, name="inspection_cover")
    for i, (x, y) in enumerate(((-0.10, -0.19), (-0.10, 0.19), (0.11, -0.19), (0.11, 0.19))):
        add_bolt_z(hip, f"top_bolt_{i}", (x, y, 0.225))

    # Upper limb link: a chunky armor shell with service covers, side rails, and
    # a forked knee casting.  The frame remains on the hip shaft.
    thigh = model.part("thigh")
    geom, orient = cyl_y(0.075, 0.26)
    thigh.visual(geom, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=orient.rpy), material=steel, name="hip_hub")
    thigh.visual(Box((0.13, 0.22, 0.20)), origin=Origin(xyz=(0.02, 0.0, -0.155)), material=dark, name="upper_web")
    thigh.visual(Box((0.18, 0.26, 0.37)), origin=Origin(xyz=(0.02, 0.0, -0.315)), material=armor, name="thigh_shell")
    thigh.visual(Box((0.05, 0.19, 0.30)), origin=Origin(xyz=(0.132, 0.0, -0.325)), material=service, name="actuator_bay")
    thigh.visual(Box((0.09, 0.036, 0.39)), origin=Origin(xyz=(0.015, 0.145, -0.39)), material=dark, name="side_rail_0")
    thigh.visual(Box((0.09, 0.036, 0.39)), origin=Origin(xyz=(0.015, -0.145, -0.39)), material=dark, name="side_rail_1")
    thigh.visual(Box((0.18, 0.055, 0.23)), origin=Origin(xyz=(0.02, 0.16, -0.62)), material=dark, name="knee_fork_0")
    thigh.visual(Box((0.18, 0.055, 0.23)), origin=Origin(xyz=(0.02, -0.16, -0.62)), material=dark, name="knee_fork_1")
    geom, orient = cyl_y(0.045, 0.42)
    thigh.visual(geom, origin=Origin(xyz=(0.02, 0.0, -0.62), rpy=orient.rpy), material=steel, name="knee_pin")
    geom, orient = cyl_y(0.070, 0.035)
    thigh.visual(geom, origin=Origin(xyz=(0.02, 0.225, -0.62), rpy=orient.rpy), material=steel, name="knee_cap_0")
    thigh.visual(geom, origin=Origin(xyz=(0.02, -0.225, -0.62), rpy=orient.rpy), material=steel, name="knee_cap_1")
    for i, (y, z) in enumerate(((-0.065, -0.235), (0.065, -0.235), (-0.065, -0.455), (0.065, -0.455))):
        add_bolt_x(thigh, f"bay_bolt_{i}", (0.158, y, z), radius=0.010)

    # Lower limb link: a tapered service shell with a separate ankle yoke and
    # rubber-lined abrasion guards on the front edge.
    shin = model.part("shin")
    geom, orient = cyl_y(0.072, 0.24)
    shin.visual(geom, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=orient.rpy), material=steel, name="knee_hub")
    shin.visual(Box((0.12, 0.20, 0.17)), origin=Origin(xyz=(0.025, 0.0, -0.14)), material=dark, name="upper_web")
    shin.visual(Box((0.16, 0.22, 0.37)), origin=Origin(xyz=(0.035, 0.0, -0.315)), material=armor, name="shin_shell")
    shin.visual(Box((0.045, 0.18, 0.30)), origin=Origin(xyz=(-0.067, 0.0, -0.32)), material=service, name="drive_cover")
    shin.visual(Box((0.035, 0.16, 0.30)), origin=Origin(xyz=(0.128, 0.0, -0.35)), material=rubber, name="front_wear_strip")
    shin.visual(Box((0.08, 0.04, 0.36)), origin=Origin(xyz=(0.035, 0.128, -0.39)), material=dark, name="edge_rail_0")
    shin.visual(Box((0.08, 0.04, 0.36)), origin=Origin(xyz=(0.035, -0.128, -0.39)), material=dark, name="edge_rail_1")
    shin.visual(Box((0.16, 0.050, 0.22)), origin=Origin(xyz=(0.04, 0.15, -0.58)), material=dark, name="ankle_fork_0")
    shin.visual(Box((0.16, 0.050, 0.22)), origin=Origin(xyz=(0.04, -0.15, -0.58)), material=dark, name="ankle_fork_1")
    geom, orient = cyl_y(0.040, 0.38)
    shin.visual(geom, origin=Origin(xyz=(0.04, 0.0, -0.58), rpy=orient.rpy), material=steel, name="ankle_pin")
    for i, (y, z) in enumerate(((-0.065, -0.25), (0.065, -0.25), (-0.065, -0.43), (0.065, -0.43))):
        add_bolt_x(shin, f"cover_bolt_{i}", (-0.096, y, z), radius=0.009)

    # Foot: broad field-service boot with replaceable rubber pads and a guarded
    # ankle hub.  The frame sits on the ankle pitch axis.
    foot = model.part("foot")
    geom, orient = cyl_y(0.065, 0.22)
    foot.visual(geom, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=orient.rpy), material=steel, name="ankle_hub")
    foot.visual(Box((0.12, 0.17, 0.15)), origin=Origin(xyz=(0.02, 0.0, -0.12)), material=dark, name="ankle_neck")
    foot.visual(Box((0.54, 0.26, 0.10)), origin=Origin(xyz=(0.12, 0.0, -0.22)), material=dark, name="sole_carrier")
    foot.visual(Box((0.20, 0.25, 0.045)), origin=Origin(xyz=(-0.08, 0.0, -0.288)), material=rubber, name="heel_wear_pad")
    foot.visual(Box((0.26, 0.25, 0.045)), origin=Origin(xyz=(0.24, 0.0, -0.288)), material=rubber, name="toe_wear_pad")
    foot.visual(Box((0.10, 0.22, 0.055)), origin=Origin(xyz=(0.405, 0.0, -0.215)), material=armor, name="toe_bumper")
    for i, (x, y) in enumerate(((0.02, -0.09), (0.02, 0.09), (0.28, -0.09), (0.28, 0.09))):
        add_bolt_z(foot, f"sole_bolt_{i}", (x, y, -0.164), radius=0.010)

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip,
        child=thigh,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=900.0, velocity=1.2, lower=-0.75, upper=0.85),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shin,
        origin=Origin(xyz=(0.02, 0.0, -0.62)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1050.0, velocity=1.4, lower=0.0, upper=1.55),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shin,
        child=foot,
        origin=Origin(xyz=(0.04, 0.0, -0.58)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=1.6, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip = object_model.get_part("hip_mount")
    thigh = object_model.get_part("thigh")
    shin = object_model.get_part("shin")
    foot = object_model.get_part("foot")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    ctx.allow_overlap(
        hip,
        thigh,
        elem_a="hip_pin",
        elem_b="hip_hub",
        reason="The modeled hip shaft intentionally passes through the replaceable bearing hub.",
    )
    ctx.expect_overlap(hip, thigh, axes="xz", elem_a="hip_pin", elem_b="hip_hub", min_overlap=0.08, name="hip pin crosses hub bore")
    ctx.expect_within(thigh, hip, axes="y", inner_elem="hip_hub", outer_elem="hip_pin", margin=0.001, name="hip hub captured on shaft length")

    ctx.allow_overlap(
        thigh,
        shin,
        elem_a="knee_pin",
        elem_b="knee_hub",
        reason="The knee shaft is captured through the shin hub as a real pinned revolute joint.",
    )
    ctx.expect_overlap(thigh, shin, axes="xz", elem_a="knee_pin", elem_b="knee_hub", min_overlap=0.075, name="knee pin crosses hub bore")
    ctx.expect_within(shin, thigh, axes="y", inner_elem="knee_hub", outer_elem="knee_pin", margin=0.001, name="knee hub captured on shaft length")

    ctx.allow_overlap(
        shin,
        foot,
        elem_a="ankle_pin",
        elem_b="ankle_hub",
        reason="The ankle shaft is intentionally nested through the foot hub bearing.",
    )
    ctx.expect_overlap(shin, foot, axes="xz", elem_a="ankle_pin", elem_b="ankle_hub", min_overlap=0.07, name="ankle pin crosses hub bore")
    ctx.expect_within(foot, shin, axes="y", inner_elem="ankle_hub", outer_elem="ankle_pin", margin=0.001, name="ankle hub captured on shaft length")

    ctx.expect_origin_gap(hip, shin, axis="z", min_gap=0.55, name="knee hangs below hip in service stance")
    ctx.expect_origin_gap(shin, foot, axis="z", min_gap=0.52, name="ankle hangs below knee in service stance")

    rest_knee = ctx.part_world_position(shin)
    rest_ankle = ctx.part_world_position(foot)
    with ctx.pose({hip_pitch: -0.45}):
        swung_knee = ctx.part_world_position(shin)
    with ctx.pose({knee_pitch: 1.0}):
        flexed_ankle = ctx.part_world_position(foot)
    with ctx.pose({ankle_pitch: -0.45}):
        toe_box = ctx.part_element_world_aabb(foot, elem="toe_wear_pad")
    rest_toe_box = ctx.part_element_world_aabb(foot, elem="toe_wear_pad")

    ctx.check(
        "hip pitch swings knee forward",
        rest_knee is not None and swung_knee is not None and swung_knee[0] > rest_knee[0] + 0.20,
        details=f"rest_knee={rest_knee}, swung_knee={swung_knee}",
    )
    ctx.check(
        "knee flexion drives ankle forward",
        rest_ankle is not None and flexed_ankle is not None and flexed_ankle[0] > rest_ankle[0] + 0.35,
        details=f"rest_ankle={rest_ankle}, flexed_ankle={flexed_ankle}",
    )
    ctx.check(
        "ankle dorsiflexion raises toe pad",
        rest_toe_box is not None and toe_box is not None and toe_box[0][2] > rest_toe_box[0][2] + 0.05,
        details=f"rest_toe_aabb={rest_toe_box}, flexed_toe_aabb={toe_box}",
    )

    return ctx.report()


object_model = build_object_model()
