from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="campanile_clock_tower")

    stone = model.material("stone", rgba=(0.84, 0.81, 0.74, 1.0))
    weathered_stone = model.material("weathered_stone", rgba=(0.73, 0.70, 0.64, 1.0))
    roof_metal = model.material("roof_metal", rgba=(0.31, 0.34, 0.38, 1.0))
    dial_white = model.material("dial_white", rgba=(0.94, 0.93, 0.89, 1.0))
    hand_black = model.material("hand_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bell_bronze = model.material("bell_bronze", rgba=(0.56, 0.39, 0.17, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=3.40, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        material=stone,
        name="lower_plinth",
    )
    base.visual(
        Cylinder(radius=2.80, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 1.40)),
        material=stone,
        name="upper_plinth",
    )

    shaft = model.part("shaft")
    shaft.visual(
        Cylinder(radius=2.10, length=16.00),
        origin=Origin(xyz=(0.0, 0.0, 8.00)),
        material=stone,
        name="tower_shaft",
    )
    shaft.visual(
        Cylinder(radius=2.35, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=weathered_stone,
        name="shaft_collar",
    )
    shaft.visual(
        Box((0.95, 0.14, 2.40)),
        origin=Origin(xyz=(0.0, 2.03, 2.70)),
        material=weathered_stone,
        name="entry_reveal",
    )

    belfry_frame = model.part("belfry_frame")
    belfry_frame.visual(
        Cylinder(radius=2.55, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=weathered_stone,
        name="lower_belfry_ring",
    )
    belfry_frame.visual(
        Cylinder(radius=2.75, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 4.15)),
        material=weathered_stone,
        name="upper_belfry_ring",
    )
    for index in range(8):
        angle = (index + 0.5) * (math.tau / 8.0)
        belfry_frame.visual(
            Cylinder(radius=0.22, length=3.30),
            origin=Origin(
                xyz=(
                    2.15 * math.cos(angle),
                    2.15 * math.sin(angle),
                    2.35,
                )
            ),
            material=stone,
            name=f"belfry_column_{index + 1}",
        )
    belfry_frame.visual(
        Box((1.95, 0.18, 2.70)),
        origin=Origin(xyz=(0.0, 2.13, 2.35)),
        material=weathered_stone,
        name="clock_panel",
    )

    roof = model.part("roof")
    roof.visual(
        Cylinder(radius=2.45, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=roof_metal,
        name="roof_drum",
    )
    roof.visual(
        mesh_from_geometry(
            ConeGeometry(radius=2.35, height=2.80, radial_segments=48, closed=True),
            "campanile_roof_cone",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.65)),
        material=roof_metal,
        name="roof_cone",
    )
    roof.visual(
        Cylinder(radius=0.10, length=0.45),
        origin=Origin(xyz=(0.0, 0.0, 3.275)),
        material=roof_metal,
        name="finial_post",
    )
    roof.visual(
        Sphere(radius=0.18),
        origin=Origin(xyz=(0.0, 0.0, 3.59)),
        material=roof_metal,
        name="finial_ball",
    )

    bell_shell = ConeGeometry(radius=0.70, height=1.20, radial_segments=40, closed=True)
    bell_shell.rotate_x(math.pi)

    bell_assembly = model.part("bell_assembly")
    bell_assembly.visual(
        Box((1.60, 0.24, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 3.60)),
        material=weathered_stone,
        name="bell_beam",
    )
    bell_assembly.visual(
        Cylinder(radius=0.08, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 3.36)),
        material=roof_metal,
        name="bell_hanger",
    )
    bell_assembly.visual(
        mesh_from_geometry(bell_shell, "campanile_bell_shell"),
        origin=Origin(xyz=(0.0, 0.0, 2.62)),
        material=bell_bronze,
        name="bell_shell",
    )

    clock_face = model.part("clock_face")
    clock_face.visual(
        Cylinder(radius=1.24, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.06)),
        material=hand_black,
        name="dial_bezel",
    )
    clock_face.visual(
        Cylinder(radius=1.16, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
        material=dial_white,
        name="dial_disc",
    )
    clock_face.visual(
        Box((0.12, 0.28, 0.01)),
        origin=Origin(xyz=(0.0, -0.80, 0.005)),
        material=hand_black,
        name="mark_12",
    )
    clock_face.visual(
        Box((0.12, 0.28, 0.01)),
        origin=Origin(xyz=(0.0, 0.80, 0.005)),
        material=hand_black,
        name="mark_6",
    )
    clock_face.visual(
        Box((0.28, 0.12, 0.01)),
        origin=Origin(xyz=(0.80, 0.0, 0.005)),
        material=hand_black,
        name="mark_3",
    )
    clock_face.visual(
        Box((0.28, 0.12, 0.01)),
        origin=Origin(xyz=(-0.80, 0.0, 0.005)),
        material=hand_black,
        name="mark_9",
    )
    clock_face.visual(
        Cylinder(radius=0.12, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=hand_black,
        name="dial_boss",
    )

    hour_hand = model.part("hour_hand")
    hour_hand.visual(
        Cylinder(radius=0.09, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=hand_black,
        name="hour_hub",
    )
    hour_hand.visual(
        Box((0.12, 0.84, 0.014)),
        origin=Origin(xyz=(0.0, -0.45, 0.078)),
        material=hand_black,
        name="hour_blade",
    )
    hour_hand.visual(
        Sphere(radius=0.04),
        origin=Origin(xyz=(0.0, -0.89, 0.078)),
        material=hand_black,
        name="hour_tip",
    )
    hour_hand.visual(
        Box((0.08, 0.18, 0.014)),
        origin=Origin(xyz=(0.0, 0.16, 0.078)),
        material=hand_black,
        name="hour_counterweight",
    )

    minute_hand = model.part("minute_hand")
    minute_hand.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=hand_black,
        name="minute_hub",
    )
    minute_hand.visual(
        Box((0.08, 1.08, 0.012)),
        origin=Origin(xyz=(0.0, -0.57, 0.155)),
        material=hand_black,
        name="minute_blade",
    )
    minute_hand.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.0, -1.13, 0.155)),
        material=hand_black,
        name="minute_tip",
    )
    minute_hand.visual(
        Box((0.06, 0.22, 0.012)),
        origin=Origin(xyz=(0.0, 0.14, 0.155)),
        material=hand_black,
        name="minute_counterweight",
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.FIXED,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, 2.00)),
    )
    model.articulation(
        "shaft_to_belfry",
        ArticulationType.FIXED,
        parent=shaft,
        child=belfry_frame,
        origin=Origin(xyz=(0.0, 0.0, 16.00)),
    )
    model.articulation(
        "belfry_to_roof",
        ArticulationType.FIXED,
        parent=belfry_frame,
        child=roof,
        origin=Origin(xyz=(0.0, 0.0, 4.60)),
    )
    model.articulation(
        "belfry_to_bell",
        ArticulationType.FIXED,
        parent=belfry_frame,
        child=bell_assembly,
        origin=Origin(),
    )
    model.articulation(
        "belfry_to_clock_face",
        ArticulationType.FIXED,
        parent=belfry_frame,
        child=clock_face,
        origin=Origin(xyz=(0.0, 2.30, 2.45), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    model.articulation(
        "clock_face_to_hour_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_face,
        child=hour_hand,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.0),
    )
    model.articulation(
        "clock_face_to_minute_hand",
        ArticulationType.CONTINUOUS,
        parent=clock_face,
        child=minute_hand,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    shaft = object_model.get_part("shaft")
    belfry_frame = object_model.get_part("belfry_frame")
    roof = object_model.get_part("roof")
    clock_face = object_model.get_part("clock_face")
    hour_hand = object_model.get_part("hour_hand")
    minute_hand = object_model.get_part("minute_hand")

    hour_joint = object_model.get_articulation("clock_face_to_hour_hand")
    minute_joint = object_model.get_articulation("clock_face_to_minute_hand")

    ctx.expect_contact(base, shaft, contact_tol=0.002, name="shaft bears on the stone base")
    ctx.expect_contact(shaft, belfry_frame, contact_tol=0.002, name="belfry frame sits on the shaft")
    ctx.expect_contact(belfry_frame, roof, contact_tol=0.002, name="roof sits on the belfry crown")
    ctx.expect_contact(clock_face, belfry_frame, contact_tol=0.005, name="clock face mounts to the belfry panel")
    ctx.expect_contact(
        clock_face,
        hour_hand,
        elem_a="dial_boss",
        elem_b="hour_hub",
        contact_tol=0.001,
        name="hour hand hub bears on the central clock boss",
    )
    ctx.expect_contact(
        hour_hand,
        minute_hand,
        elem_a="hour_hub",
        elem_b="minute_hub",
        contact_tol=0.001,
        name="minute hand hub stacks on the hour hand hub",
    )

    ctx.check(
        "clock hands use coaxial continuous joints",
        hour_joint.articulation_type == ArticulationType.CONTINUOUS
        and minute_joint.articulation_type == ArticulationType.CONTINUOUS
        and hour_joint.axis == minute_joint.axis
        and hour_joint.origin.xyz == minute_joint.origin.xyz
        and hour_joint.origin.rpy == minute_joint.origin.rpy,
        details=(
            f"hour_type={hour_joint.articulation_type}, minute_type={minute_joint.articulation_type}, "
            f"hour_axis={hour_joint.axis}, minute_axis={minute_joint.axis}, "
            f"hour_origin={hour_joint.origin}, minute_origin={minute_joint.origin}"
        ),
    )

    def elem_center(part_obj, elem_name):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((a + b) / 2.0 for a, b in zip(lower, upper))

    def dial_radius(point, axle):
        if point is None or axle is None:
            return None
        return math.hypot(point[0] - axle[0], point[2] - axle[2])

    axle = ctx.part_world_position(clock_face)
    hour_tip_rest = elem_center(hour_hand, "hour_tip")
    minute_tip_rest = elem_center(minute_hand, "minute_tip")

    with ctx.pose({hour_joint: math.pi / 3.0}):
        hour_tip_turned = elem_center(hour_hand, "hour_tip")

    with ctx.pose({minute_joint: math.pi / 2.0}):
        minute_tip_turned = elem_center(minute_hand, "minute_tip")

    hour_ok = (
        axle is not None
        and hour_tip_rest is not None
        and hour_tip_turned is not None
        and abs(hour_tip_turned[1] - hour_tip_rest[1]) < 0.02
        and math.dist(hour_tip_rest, hour_tip_turned) > 0.45
        and abs(dial_radius(hour_tip_rest, axle) - dial_radius(hour_tip_turned, axle)) < 0.03
    )
    minute_ok = (
        axle is not None
        and minute_tip_rest is not None
        and minute_tip_turned is not None
        and abs(minute_tip_turned[1] - minute_tip_rest[1]) < 0.02
        and math.dist(minute_tip_rest, minute_tip_turned) > 0.75
        and abs(dial_radius(minute_tip_rest, axle) - dial_radius(minute_tip_turned, axle)) < 0.03
    )

    ctx.check(
        "hour hand tip sweeps around the dial",
        hour_ok,
        details=f"axle={axle}, rest={hour_tip_rest}, turned={hour_tip_turned}",
    )
    ctx.check(
        "minute hand tip sweeps around the dial",
        minute_ok,
        details=f"axle={axle}, rest={minute_tip_rest}, turned={minute_tip_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
