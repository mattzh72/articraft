from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


def _x_cyl(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose local Z axis is laid along object +X."""
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _y_cyl(x: float, y: float, z: float) -> Origin:
    """Origin for a cylinder whose local Z axis is laid along object Y."""
    return Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0))


def _helical_spring(
    *,
    start_x: float,
    end_x: float,
    center_y: float,
    center_z: float,
    coil_radius: float,
    wire_radius: float,
    turns: int,
):
    points = []
    samples = turns * 18 + 1
    for i in range(samples):
        u = i / (samples - 1)
        theta = 2.0 * math.pi * turns * u
        points.append(
            (
                start_x + (end_x - start_x) * u,
                center_y + coil_radius * math.cos(theta),
                center_z + coil_radius * math.sin(theta),
            )
        )
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=3,
        radial_segments=10,
        cap_ends=True,
        up_hint=(0.0, 0.0, 1.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_microphone_boom")

    matte_black = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.055, 0.052, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    grille = model.material("silver_grille", rgba=(0.78, 0.80, 0.76, 1.0))
    wood = model.material("warm_desk_edge", rgba=(0.58, 0.36, 0.18, 1.0))

    root = model.part("desk_clamp")
    # A short desktop edge is included so the C-clamp reads as desk-mounted.
    root.visual(
        Box((0.30, 0.18, 0.040)),
        origin=Origin(xyz=(-0.090, 0.0, -0.300)),
        material=wood,
        name="desk_edge",
    )
    root.visual(
        Box((0.050, 0.105, 0.225)),
        origin=Origin(xyz=(-0.010, 0.0, -0.270)),
        material=matte_black,
        name="clamp_spine",
    )
    root.visual(
        Box((0.135, 0.112, 0.018)),
        origin=Origin(xyz=(-0.068, 0.0, -0.273)),
        material=matte_black,
        name="upper_jaw",
    )
    root.visual(
        Box((0.125, 0.105, 0.018)),
        origin=Origin(xyz=(-0.066, 0.0, -0.333)),
        material=matte_black,
        name="lower_jaw",
    )
    root.visual(
        Cylinder(radius=0.015, length=0.060),
        origin=Origin(xyz=(-0.070, 0.0, -0.365)),
        material=steel,
        name="clamp_screw",
    )
    root.visual(
        Cylinder(radius=0.035, length=0.015),
        origin=Origin(xyz=(-0.070, 0.0, -0.325)),
        material=dark_rubber,
        name="pressure_pad",
    )
    root.visual(
        Cylinder(radius=0.035, length=0.020),
        origin=Origin(xyz=(-0.070, 0.0, -0.400)),
        material=satin_black,
        name="tightening_knob",
    )
    root.visual(
        Cylinder(radius=0.024, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=matte_black,
        name="vertical_post",
    )
    root.visual(
        Cylinder(radius=0.042, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=satin_black,
        name="turret_cap",
    )

    lower = model.part("lower_arm")
    lower.visual(
        Cylinder(radius=0.035, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=satin_black,
        name="swivel_sleeve",
    )
    lower.visual(
        Cylinder(radius=0.018, length=0.094),
        origin=_y_cyl(0.048, 0.0, 0.045),
        material=satin_black,
        name="shoulder_crossbar",
    )
    for idx, y in enumerate((-0.035, 0.035)):
        lower.visual(
            Cylinder(radius=0.010, length=0.520),
            origin=_x_cyl(0.300, y, 0.045),
            material=matte_black,
            name=f"lower_tube_{idx}",
        )
    lower.visual(
        Cylinder(radius=0.013, length=0.100),
        origin=_y_cyl(0.525, 0.0, 0.045),
        material=satin_black,
        name="elbow_crossbar",
    )
    lower.visual(
        Box((0.068, 0.012, 0.070)),
        origin=Origin(xyz=(0.580, -0.045, 0.045)),
        material=satin_black,
        name="elbow_plate_0",
    )
    lower.visual(
        Box((0.068, 0.012, 0.070)),
        origin=Origin(xyz=(0.580, 0.045, 0.045)),
        material=satin_black,
        name="elbow_plate_1",
    )
    lower.visual(
        mesh_from_geometry(
            _helical_spring(
                start_x=0.135,
                end_x=0.485,
                center_y=0.0,
                center_z=0.083,
                coil_radius=0.010,
                wire_radius=0.0023,
                turns=14,
            ),
            "lower_spring",
        ),
        material=steel,
        name="lower_spring",
    )
    for idx, x in enumerate((0.135, 0.485)):
        lower.visual(
            Box((0.020, 0.082, 0.032)),
            origin=Origin(xyz=(x, 0.0, 0.062)),
            material=satin_black,
            name=f"lower_spring_anchor_{idx}",
        )

    upper = model.part("upper_arm")
    upper.visual(
        Cylinder(radius=0.024, length=0.078),
        origin=_y_cyl(0.0, 0.0, 0.0),
        material=satin_black,
        name="elbow_hub",
    )
    for idx, y in enumerate((-0.030, 0.030)):
        upper.visual(
            Cylinder(radius=0.0085, length=0.420),
            origin=_x_cyl(0.230, y, 0.0),
            material=matte_black,
            name=f"upper_tube_{idx}",
        )
    upper.visual(
        Cylinder(radius=0.012, length=0.086),
        origin=_y_cyl(0.420, 0.0, 0.0),
        material=satin_black,
        name="wrist_crossbar",
    )
    upper.visual(
        Box((0.058, 0.012, 0.060)),
        origin=Origin(xyz=(0.480, -0.040, 0.0)),
        material=satin_black,
        name="wrist_plate_0",
    )
    upper.visual(
        Box((0.058, 0.012, 0.060)),
        origin=Origin(xyz=(0.480, 0.040, 0.0)),
        material=satin_black,
        name="wrist_plate_1",
    )
    for idx, y in enumerate((-0.040, 0.040)):
        upper.visual(
            Box((0.060, 0.012, 0.024)),
            origin=Origin(xyz=(0.440, y, -0.018)),
            material=satin_black,
            name=f"wrist_plate_bridge_{idx}",
        )
    upper.visual(
        mesh_from_geometry(
            _helical_spring(
                start_x=0.075,
                end_x=0.370,
                center_y=0.0,
                center_z=0.040,
                coil_radius=0.008,
                wire_radius=0.0020,
                turns=11,
            ),
            "upper_spring",
        ),
        material=steel,
        name="upper_spring",
    )
    for idx, x in enumerate((0.075, 0.370)):
        upper.visual(
            Box((0.018, 0.070, 0.052)),
            origin=Origin(xyz=(x, 0.0, 0.016)),
            material=satin_black,
            name=f"upper_spring_anchor_{idx}",
        )

    mic = model.part("microphone")
    mic.visual(
        Cylinder(radius=0.020, length=0.068),
        origin=_y_cyl(0.0, 0.0, 0.0),
        material=satin_black,
        name="wrist_hub",
    )
    mic.visual(
        Cylinder(radius=0.010, length=0.086),
        origin=_x_cyl(0.043, 0.0, 0.0),
        material=satin_black,
        name="short_stem",
    )
    mic.visual(
        Cylinder(radius=0.028, length=0.145),
        origin=_x_cyl(0.150, 0.0, 0.0),
        material=dark_rubber,
        name="mic_body",
    )
    mic.visual(
        Cylinder(radius=0.030, length=0.075),
        origin=_x_cyl(0.255, 0.0, 0.0),
        material=grille,
        name="front_grille",
    )
    for idx, x in enumerate((0.220, 0.245, 0.270, 0.292)):
        mic.visual(
            Cylinder(radius=0.0315, length=0.006),
            origin=_x_cyl(x, 0.0, 0.0),
            material=matte_black if idx in (0, 3) else steel,
            name=f"grille_band_{idx}",
        )
    mic.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=_x_cyl(0.075, 0.0, 0.0),
        material=steel,
        name="mount_collar",
    )

    yaw = model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=root,
        child=lower,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.5, lower=-2.7, upper=2.7),
    )
    elbow = model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.580, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.75, upper=1.25),
    )
    wrist = model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=upper,
        child=mic,
        origin=Origin(xyz=(0.480, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-1.0, upper=1.0),
    )
    # Store stable names for prompt-specific tests and downstream inspection.
    model.meta["primary_joints"] = (yaw.name, elbow.name, wrist.name)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    yaw = object_model.get_articulation("base_yaw")
    elbow = object_model.get_articulation("elbow_pitch")
    wrist = object_model.get_articulation("wrist_pitch")
    lower = object_model.get_part("lower_arm")
    upper = object_model.get_part("upper_arm")
    mic = object_model.get_part("microphone")

    primary = [yaw, elbow, wrist]
    ctx.check(
        "three serial revolute joints",
        len(primary) == 3
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in primary)
        and [j.parent for j in primary] == ["desk_clamp", "lower_arm", "upper_arm"]
        and [j.child for j in primary] == ["lower_arm", "upper_arm", "microphone"],
        details=f"{[(j.name, j.parent, j.child, j.articulation_type) for j in primary]}",
    )

    ctx.expect_gap(
        lower,
        "desk_clamp",
        axis="z",
        positive_elem="swivel_sleeve",
        negative_elem="turret_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel sleeve seats on turret cap",
    )
    ctx.expect_gap(
        "lower_arm",
        upper,
        axis="y",
        positive_elem="elbow_plate_1",
        negative_elem="elbow_hub",
        max_gap=0.001,
        max_penetration=0.0005,
        name="elbow hub clears upper clevis cheek",
    )
    ctx.expect_gap(
        upper,
        "lower_arm",
        axis="y",
        positive_elem="elbow_hub",
        negative_elem="elbow_plate_0",
        max_gap=0.001,
        max_penetration=0.0005,
        name="elbow hub clears lower clevis cheek",
    )
    ctx.expect_gap(
        "upper_arm",
        mic,
        axis="y",
        positive_elem="wrist_plate_1",
        negative_elem="wrist_hub",
        max_gap=0.001,
        max_penetration=0.0005,
        name="microphone trunnion clears upper wrist cheek",
    )
    ctx.expect_gap(
        mic,
        "upper_arm",
        axis="y",
        positive_elem="wrist_hub",
        negative_elem="wrist_plate_0",
        max_gap=0.001,
        max_penetration=0.0005,
        name="microphone trunnion clears lower wrist cheek",
    )

    rest_mic_pos = ctx.part_world_position(mic)
    with ctx.pose({elbow: 0.85}):
        raised_mic_pos = ctx.part_world_position(mic)
    ctx.check(
        "elbow pitch raises the wrist end",
        rest_mic_pos is not None
        and raised_mic_pos is not None
        and raised_mic_pos[2] > rest_mic_pos[2] + 0.25,
        details=f"rest={rest_mic_pos}, raised={raised_mic_pos}",
    )

    with ctx.pose({yaw: 1.0}):
        yawed_mic_pos = ctx.part_world_position(mic)
    ctx.check(
        "base yaw swings the boom sideways",
        rest_mic_pos is not None
        and yawed_mic_pos is not None
        and abs(yawed_mic_pos[1] - rest_mic_pos[1]) > 0.60,
        details=f"rest={rest_mic_pos}, yawed={yawed_mic_pos}",
    )

    rest_grille = ctx.part_element_world_aabb(mic, elem="front_grille")
    with ctx.pose({wrist: 0.70}):
        tipped_grille = ctx.part_element_world_aabb(mic, elem="front_grille")
    rest_grille_z = None if rest_grille is None else (rest_grille[0][2] + rest_grille[1][2]) / 2.0
    tipped_grille_z = (
        None if tipped_grille is None else (tipped_grille[0][2] + tipped_grille[1][2]) / 2.0
    )
    ctx.check(
        "wrist pitch tilts microphone grille upward",
        rest_grille_z is not None
        and tipped_grille_z is not None
        and tipped_grille_z > rest_grille_z + 0.10,
        details=f"rest_z={rest_grille_z}, tipped_z={tipped_grille_z}",
    )

    return ctx.report()


object_model = build_object_model()
