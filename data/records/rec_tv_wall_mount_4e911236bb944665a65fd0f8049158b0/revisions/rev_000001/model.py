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
    model = ArticulatedObject(name="compact_tv_wall_mount")

    model.material("black_powdercoat", rgba=(0.025, 0.027, 0.030, 1.0))
    model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("edge_wear", rgba=(0.34, 0.35, 0.34, 1.0))
    model.material("zinc_screw", rgba=(0.70, 0.71, 0.68, 1.0))
    model.material("hole_shadow", rgba=(0.005, 0.005, 0.006, 1.0))

    # Object frame: first wall-side vertical pivot.  +X projects out from wall,
    # +Z is up, and +Y spans the bracket/faceplate width.
    wall = model.part("wall_bracket")
    wall.visual(
        Box((0.028, 0.220, 0.300)),
        origin=Origin(xyz=(-0.072, 0.0, 0.0)),
        material="black_powdercoat",
        name="wall_plate",
    )
    wall.visual(
        Box((0.047, 0.115, 0.165)),
        origin=Origin(xyz=(-0.054, 0.0, 0.0)),
        material="black_powdercoat",
        name="standoff_block",
    )
    for label, z in (("lower", -0.0485), ("upper", 0.0485)):
        wall.visual(
            Cylinder(radius=0.037, length=0.048),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material="dark_steel",
            name=f"wall_hinge_barrel_{label}",
        )
    for y in (-0.072, 0.072):
        for z in (-0.102, 0.102):
            wall.visual(
                Cylinder(radius=0.012, length=0.006),
                origin=Origin(xyz=(-0.055, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material="zinc_screw",
                name=f"lag_screw_{y}_{z}",
            )

    primary = model.part("primary_arm")
    primary.visual(
        Cylinder(radius=0.031, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="dark_steel",
        name="shoulder_barrel",
    )
    primary.visual(
        Box((0.215, 0.052, 0.034)),
        origin=Origin(xyz=(0.112, 0.0, 0.0)),
        material="black_powdercoat",
        name="arm_tube",
    )
    primary.visual(
        Box((0.205, 0.018, 0.010)),
        origin=Origin(xyz=(0.112, 0.034, 0.019)),
        material="edge_wear",
        name="upper_rib",
    )
    primary.visual(
        Box((0.205, 0.018, 0.010)),
        origin=Origin(xyz=(0.112, -0.034, -0.019)),
        material="edge_wear",
        name="lower_rib",
    )
    primary.visual(
        Box((0.026, 0.092, 0.034)),
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        material="black_powdercoat",
        name="elbow_crossbar",
    )
    for y in (-0.043, 0.043):
        primary.visual(
            Box((0.064, 0.018, 0.128)),
            origin=Origin(xyz=(0.250, y, 0.0)),
            material="black_powdercoat",
            name=f"elbow_fork_{'side_a' if y > 0 else 'side_b'}",
        )
    for label, z in (("lower", -0.048), ("upper", 0.048)):
        primary.visual(
            Cylinder(radius=0.035, length=0.046),
            origin=Origin(xyz=(0.280, 0.0, z)),
            material="dark_steel",
            name=f"elbow_yoke_{label}",
        )

    secondary = model.part("secondary_arm")
    secondary.visual(
        Cylinder(radius=0.030, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="dark_steel",
        name="elbow_barrel",
    )
    secondary.visual(
        Box((0.165, 0.047, 0.032)),
        origin=Origin(xyz=(0.086, 0.0, 0.0)),
        material="black_powdercoat",
        name="arm_tube",
    )
    secondary.visual(
        Box((0.155, 0.014, 0.009)),
        origin=Origin(xyz=(0.086, 0.030, 0.018)),
        material="edge_wear",
        name="upper_rib",
    )
    secondary.visual(
        Box((0.155, 0.014, 0.009)),
        origin=Origin(xyz=(0.086, -0.030, -0.018)),
        material="edge_wear",
        name="lower_rib",
    )
    secondary.visual(
        Box((0.024, 0.084, 0.032)),
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
        material="black_powdercoat",
        name="head_crossbar",
    )
    for y in (-0.039, 0.039):
        secondary.visual(
            Box((0.050, 0.016, 0.122)),
            origin=Origin(xyz=(0.189, y, 0.0)),
            material="black_powdercoat",
            name=f"head_fork_{'side_a' if y > 0 else 'side_b'}",
        )
    for z in (-0.047, 0.047):
        secondary.visual(
            Cylinder(radius=0.032, length=0.044),
            origin=Origin(xyz=(0.220, 0.0, z)),
            material="dark_steel",
            name=f"head_yoke_{'upper' if z > 0 else 'lower'}",
        )

    swivel = model.part("swivel_block")
    swivel.visual(
        Cylinder(radius=0.029, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="dark_steel",
        name="swivel_barrel",
    )
    swivel.visual(
        Box((0.044, 0.158, 0.046)),
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
        material="black_powdercoat",
        name="tilt_bridge",
    )
    for label, y in (("side_b", -0.063), ("side_a", 0.063)):
        swivel.visual(
            Box((0.068, 0.024, 0.046)),
            origin=Origin(xyz=(0.070, y, 0.0)),
            material="black_powdercoat",
            name=f"tilt_clevis_{label}",
        )
        swivel.visual(
            Cylinder(radius=0.021, length=0.038),
            origin=Origin(xyz=(0.074, y, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material="dark_steel",
            name=f"tilt_barrel_{label}",
        )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.019, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="tilt_pin_barrel",
    )
    faceplate.visual(
        Box((0.048, 0.090, 0.040)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material="black_powdercoat",
        name="rear_boss",
    )
    faceplate.visual(
        Box((0.014, 0.245, 0.185)),
        origin=Origin(xyz=(0.057, 0.0, 0.0)),
        material="black_powdercoat",
        name="vesa_plate",
    )
    faceplate.visual(
        Box((0.008, 0.155, 0.030)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material="edge_wear",
        name="horizontal_pressing",
    )
    faceplate.visual(
        Box((0.008, 0.030, 0.130)),
        origin=Origin(xyz=(0.066, 0.0, 0.0)),
        material="edge_wear",
        name="vertical_pressing",
    )
    for y in (-0.050, 0.050):
        for z in (-0.050, 0.050):
            faceplate.visual(
                Cylinder(radius=0.009, length=0.004),
                origin=Origin(xyz=(0.066, y, z), rpy=(0.0, pi / 2.0, 0.0)),
                material="hole_shadow",
                name=f"vesa_hole_{y}_{z}",
            )

    model.articulation(
        "wall_pivot",
        ArticulationType.REVOLUTE,
        parent=wall,
        child=primary,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.2, lower=-1.15, upper=1.15),
    )
    model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=26.0, velocity=1.2, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "face_swivel",
        ArticulationType.REVOLUTE,
        parent=secondary,
        child=swivel,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=faceplate,
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.30, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall = object_model.get_part("wall_bracket")
    primary = object_model.get_part("primary_arm")
    secondary = object_model.get_part("secondary_arm")
    swivel = object_model.get_part("swivel_block")
    faceplate = object_model.get_part("faceplate")

    wall_pivot = object_model.get_articulation("wall_pivot")
    elbow_pivot = object_model.get_articulation("elbow_pivot")
    face_swivel = object_model.get_articulation("face_swivel")
    tilt_hinge = object_model.get_articulation("tilt_hinge")

    ctx.check(
        "two arm joints rotate on vertical axes",
        wall_pivot.articulation_type == ArticulationType.REVOLUTE
        and elbow_pivot.articulation_type == ArticulationType.REVOLUTE
        and wall_pivot.axis == (0.0, 0.0, 1.0)
        and elbow_pivot.axis == (0.0, 0.0, 1.0),
    )
    ctx.check(
        "faceplate has swivel and tilt axes",
        face_swivel.axis == (0.0, 0.0, 1.0)
        and tilt_hinge.axis == (0.0, 1.0, 0.0)
        and abs(face_swivel.motion_limits.upper) < abs(elbow_pivot.motion_limits.upper),
    )

    ctx.expect_gap(
        wall,
        primary,
        axis="z",
        positive_elem="wall_hinge_barrel_upper",
        negative_elem="shoulder_barrel",
        max_gap=0.001,
        max_penetration=0.001,
        name="wall hinge barrels seat the shoulder knuckle",
    )
    ctx.expect_gap(
        primary,
        secondary,
        axis="z",
        positive_elem="elbow_yoke_upper",
        negative_elem="elbow_barrel",
        max_gap=0.001,
        max_penetration=0.001,
        name="elbow knuckle is captured between yoke barrels",
    )
    ctx.expect_gap(
        swivel,
        faceplate,
        axis="y",
        positive_elem="tilt_barrel_side_a",
        negative_elem="tilt_pin_barrel",
        max_gap=0.001,
        max_penetration=0.001,
        name="tilt hinge barrel is captured in the clevis",
    )

    rest_face = ctx.part_world_position(faceplate)
    with ctx.pose({wall_pivot: 0.60}):
        swung_face = ctx.part_world_position(faceplate)
    ctx.check(
        "wall pivot swings the whole arm away sideways",
        rest_face is not None and swung_face is not None and swung_face[1] > rest_face[1] + 0.25,
        details=f"rest={rest_face}, swung={swung_face}",
    )

    rest_swivel = ctx.part_world_position(swivel)
    with ctx.pose({elbow_pivot: 0.80}):
        bent_swivel = ctx.part_world_position(swivel)
    ctx.check(
        "elbow pivot folds the secondary arm",
        rest_swivel is not None and bent_swivel is not None and bent_swivel[1] > rest_swivel[1] + 0.14,
        details=f"rest={rest_swivel}, bent={bent_swivel}",
    )

    with ctx.pose({face_swivel: 0.32}):
        yawed_face = ctx.part_world_position(faceplate)
    ctx.check(
        "face swivel gives small yaw at the head",
        rest_face is not None and yawed_face is not None and yawed_face[1] > rest_face[1] + 0.018,
        details=f"rest={rest_face}, yawed={yawed_face}",
    )

    rest_plate_box = ctx.part_element_world_aabb(faceplate, elem="vesa_plate")
    with ctx.pose({tilt_hinge: 0.40}):
        tilted_plate_box = ctx.part_element_world_aabb(faceplate, elem="vesa_plate")
    rest_plate_z = None if rest_plate_box is None else (rest_plate_box[0][2] + rest_plate_box[1][2]) / 2.0
    tilted_plate_z = None if tilted_plate_box is None else (tilted_plate_box[0][2] + tilted_plate_box[1][2]) / 2.0
    ctx.check(
        "tilt hinge pitches the faceplate about a horizontal axis",
        rest_plate_z is not None and tilted_plate_z is not None and tilted_plate_z < rest_plate_z - 0.015,
        details=f"rest_z={rest_plate_z}, tilted_z={tilted_plate_z}",
    )

    return ctx.report()


object_model = build_object_model()
