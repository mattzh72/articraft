from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    hw = width / 2.0
    hh = height / 2.0
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _circle_profile(radius: float, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_monocular_microscope")

    body_paint = model.material("body_paint", rgba=(0.91, 0.92, 0.87, 1.0))
    stage_paint = model.material("stage_paint", rgba=(0.14, 0.15, 0.17, 1.0))
    metal_dark = model.material("metal_dark", rgba=(0.28, 0.29, 0.31, 1.0))
    metal_light = model.material("metal_light", rgba=(0.74, 0.76, 0.79, 1.0))
    glass = model.material("glass", rgba=(0.70, 0.80, 0.92, 0.55))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    frame = model.part("frame")

    # Base housing: a hollow shell assembled from touching wall sections so the
    # lamp compartment reads as a real cavity behind the front door.
    frame.visual(
        Box((0.230, 0.180, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_paint,
        name="base_floor",
    )
    frame.visual(
        Box((0.230, 0.180, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=body_paint,
        name="base_top",
    )
    frame.visual(
        Box((0.012, 0.180, 0.060)),
        origin=Origin(xyz=(-0.109, 0.0, 0.042)),
        material=body_paint,
        name="left_wall",
    )
    frame.visual(
        Box((0.012, 0.180, 0.060)),
        origin=Origin(xyz=(0.109, 0.0, 0.042)),
        material=body_paint,
        name="right_wall",
    )
    frame.visual(
        Box((0.206, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.084, 0.042)),
        material=body_paint,
        name="rear_wall",
    )
    frame.visual(
        Box((0.206, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, -0.084, 0.021)),
        material=body_paint,
        name="front_sill",
    )
    frame.visual(
        Box((0.206, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, -0.084, 0.066)),
        material=body_paint,
        name="front_lintel",
    )
    frame.visual(
        Box((0.038, 0.012, 0.030)),
        origin=Origin(xyz=(-0.084, -0.084, 0.045)),
        material=body_paint,
        name="front_left_jamb",
    )
    frame.visual(
        Box((0.038, 0.012, 0.030)),
        origin=Origin(xyz=(0.084, -0.084, 0.045)),
        material=body_paint,
        name="front_right_jamb",
    )

    # Door-side hinge tabs mounted to the front-left jamb.
    frame.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.066, -0.085, 0.058)),
        material=metal_dark,
        name="hinge_tab_upper",
    )
    frame.visual(
        Box((0.010, 0.012, 0.010)),
        origin=Origin(xyz=(-0.066, -0.085, 0.032)),
        material=metal_dark,
        name="hinge_tab_lower",
    )

    # Illuminator port on the deck and the stage guide support above it.
    frame.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, -0.004, 0.082)),
        material=glass,
        name="illuminator_lens",
    )
    frame.visual(
        Box((0.060, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, 0.086)),
        material=body_paint,
        name="stage_pedestal",
    )
    frame.visual(
        Box((0.082, 0.100, 0.018)),
        origin=Origin(xyz=(0.0, -0.004, 0.101)),
        material=metal_dark,
        name="guide_block",
    )

    arm_geom = sweep_profile_along_spline(
        [
            (0.098, 0.062, 0.082),
            (0.100, 0.066, 0.142),
            (0.098, 0.066, 0.198),
            (0.082, 0.062, 0.226),
            (0.048, 0.050, 0.238),
            (0.018, 0.036, 0.248),
        ],
        profile=rounded_rect_profile(0.032, 0.024, 0.006),
        samples_per_segment=14,
        cap_profile=True,
    )
    arm_geom.merge(BoxGeometry((0.038, 0.044, 0.030)).translate(0.096, 0.062, 0.088))
    arm_geom.merge(BoxGeometry((0.050, 0.028, 0.040)).translate(0.012, 0.034, 0.248))
    arm_mesh = mesh_from_geometry(arm_geom, "microscope_arm")
    frame.visual(arm_mesh, material=body_paint, name="arm_casting")

    frame.visual(
        Box((0.056, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.000, 0.255)),
        material=body_paint,
        name="head_block",
    )
    frame.visual(
        Cylinder(radius=0.020, length=0.160),
        origin=Origin(
            xyz=(0.0, 0.058, 0.335),
            rpy=(-math.pi / 4.0, 0.0, 0.0),
        ),
        material=body_paint,
        name="observation_tube",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.045),
        origin=Origin(
            xyz=(0.0, 0.129, 0.406),
            rpy=(-math.pi / 4.0, 0.0, 0.0),
        ),
        material=metal_light,
        name="eyepiece",
    )
    frame.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.150, 0.427),
            rpy=(-math.pi / 4.0, 0.0, 0.0),
        ),
        material=rubber,
        name="eyecup",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=metal_dark,
        name="turret_body",
    )
    turret.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=metal_light,
        name="turret_boss",
    )
    turret.visual(
        Cylinder(radius=0.0085, length=0.040),
        origin=Origin(xyz=(0.018, 0.000, -0.032)),
        material=metal_light,
        name="objective_long",
    )
    turret.visual(
        Cylinder(radius=0.0080, length=0.032),
        origin=Origin(
            xyz=(-0.009, 0.0156, -0.028),
            rpy=(0.0, 0.0, 0.0),
        ),
        material=metal_light,
        name="objective_medium",
    )
    turret.visual(
        Cylinder(radius=0.0075, length=0.024),
        origin=Origin(xyz=(-0.009, -0.0156, -0.024)),
        material=metal_light,
        name="objective_short",
    )

    stage_support = model.part("stage_support")
    # Mechanical stage carriage: a shallow channel that bears directly on the
    # guide block side faces while remaining bridged above it.
    stage_sleeve_geom = BoxGeometry((0.010, 0.108, 0.024))
    stage_sleeve_geom.translate(-0.046, 0.0, 0.001)
    stage_sleeve_geom.merge(BoxGeometry((0.010, 0.108, 0.024)).translate(0.046, 0.0, 0.001))
    stage_sleeve_geom.merge(BoxGeometry((0.102, 0.012, 0.006)).translate(0.0, -0.048, 0.013))
    stage_sleeve_geom.merge(BoxGeometry((0.102, 0.012, 0.006)).translate(0.0, 0.048, 0.013))
    stage_sleeve = mesh_from_geometry(stage_sleeve_geom, "stage_sleeve")
    stage_support.visual(
        stage_sleeve,
        material=metal_dark,
        name="stage_sleeve",
    )
    stage_plate = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(0.158, 0.142),
            [_circle_profile(0.013, segments=28)],
            0.006,
            cap=True,
            center=True,
        ),
        "stage_plate",
    )
    stage_support.visual(
        stage_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=stage_paint,
        name="stage_plate",
    )
    stage_support.visual(
        Box((0.116, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.048, 0.024)),
        material=metal_light,
        name="slide_holder_bar",
    )
    stage_support.visual(
        Box((0.010, 0.072, 0.010)),
        origin=Origin(xyz=(0.074, 0.0, 0.011)),
        material=metal_dark,
        name="stage_side_rail",
    )

    lamp_door = model.part("lamp_door")
    lamp_door.visual(
        Box((0.132, 0.004, 0.034)),
        origin=Origin(xyz=(0.066, -0.002, 0.0)),
        material=body_paint,
        name="door_panel",
    )
    lamp_door.visual(
        Box((0.016, 0.008, 0.008)),
        origin=Origin(xyz=(0.104, -0.008, 0.0)),
        material=metal_light,
        name="door_handle",
    )
    lamp_door.visual(
        Box((0.008, 0.010, 0.018)),
        origin=Origin(xyz=(0.002, -0.005, 0.0)),
        material=metal_dark,
        name="door_lug",
    )

    model.articulation(
        "head_to_turret",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=2.0 * math.pi,
        ),
    )
    model.articulation(
        "frame_to_stage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=stage_support,
        origin=Origin(xyz=(0.0, -0.004, 0.101)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.08,
            lower=-0.018,
            upper=0.018,
        ),
    )
    model.articulation(
        "frame_to_lamp_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lamp_door,
        origin=Origin(xyz=(-0.066, -0.090, 0.045)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    turret = object_model.get_part("turret")
    stage_support = object_model.get_part("stage_support")
    lamp_door = object_model.get_part("lamp_door")

    turret_joint = object_model.get_articulation("head_to_turret")
    stage_joint = object_model.get_articulation("frame_to_stage")
    door_joint = object_model.get_articulation("frame_to_lamp_door")

    ctx.check(
        "all microscope parts exist",
        all(part is not None for part in (frame, turret, stage_support, lamp_door)),
    )
    ctx.check(
        "microscope joints use requested mechanisms",
        turret_joint.articulation_type == ArticulationType.REVOLUTE
        and stage_joint.articulation_type == ArticulationType.PRISMATIC
        and door_joint.articulation_type == ArticulationType.REVOLUTE,
        details=(
            f"turret={turret_joint.articulation_type}, "
            f"stage={stage_joint.articulation_type}, "
            f"door={door_joint.articulation_type}"
        ),
    )

    ctx.expect_contact(
        turret,
        frame,
        elem_a="turret_body",
        elem_b="head_block",
        name="turret seats against the head block",
    )

    with ctx.pose({stage_joint: 0.0}):
        ctx.expect_contact(
            frame,
            stage_support,
            elem_a="guide_block",
            elem_b="stage_sleeve",
            name="stage carriage bears on the guide block at rest",
        )
        ctx.expect_within(
            frame,
            stage_support,
            axes="xz",
            inner_elem="guide_block",
            outer_elem="stage_sleeve",
            margin=0.008,
            name="guide block stays within stage sleeve in cross-section",
        )
        ctx.expect_overlap(
            frame,
            stage_support,
            axes="y",
            elem_a="guide_block",
            elem_b="stage_sleeve",
            min_overlap=0.090,
            name="stage sleeve remains substantially engaged on guide block at rest",
        )
        rest_stage_pos = ctx.part_world_position(stage_support)

    with ctx.pose({stage_joint: 0.018}):
        ctx.expect_contact(
            frame,
            stage_support,
            elem_a="guide_block",
            elem_b="stage_sleeve",
            name="stage carriage remains guided in contact at full aft travel",
        )
        ctx.expect_within(
            frame,
            stage_support,
            axes="xz",
            inner_elem="guide_block",
            outer_elem="stage_sleeve",
            margin=0.008,
            name="guide block stays within stage sleeve at full aft travel",
        )
        ctx.expect_overlap(
            frame,
            stage_support,
            axes="y",
            elem_a="guide_block",
            elem_b="stage_sleeve",
            min_overlap=0.072,
            name="stage sleeve retains insertion on guide block at full aft travel",
        )
        aft_stage_pos = ctx.part_world_position(stage_support)

    ctx.check(
        "stage translates aft along its guide",
        rest_stage_pos is not None
        and aft_stage_pos is not None
        and aft_stage_pos[1] > rest_stage_pos[1] + 0.015,
        details=f"rest={rest_stage_pos}, aft={aft_stage_pos}",
    )

    with ctx.pose({door_joint: 0.0}):
        closed_door = ctx.part_element_world_aabb(lamp_door, elem="door_panel")
    with ctx.pose({door_joint: 1.10}):
        open_door = ctx.part_element_world_aabb(lamp_door, elem="door_panel")

    def _aabb_center_y(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][1] + aabb[1][1])

    ctx.check(
        "lamp door opens outward from the base front",
        closed_door is not None
        and open_door is not None
        and _aabb_center_y(open_door) < _aabb_center_y(closed_door) - 0.030,
        details=f"closed={closed_door}, open={open_door}",
    )

    with ctx.pose({turret_joint: 0.0}):
        objective_rest = ctx.part_element_world_aabb(turret, elem="objective_long")
    with ctx.pose({turret_joint: 2.0 * math.pi / 3.0}):
        objective_indexed = ctx.part_element_world_aabb(turret, elem="objective_long")

    def _aabb_center_xy(aabb):
        if aabb is None:
            return None
        return (
            0.5 * (aabb[0][0] + aabb[1][0]),
            0.5 * (aabb[0][1] + aabb[1][1]),
        )

    rest_xy = _aabb_center_xy(objective_rest)
    indexed_xy = _aabb_center_xy(objective_indexed)
    ctx.check(
        "turret rotation reindexes the objective positions around the optical axis",
        rest_xy is not None
        and indexed_xy is not None
        and math.hypot(indexed_xy[0] - rest_xy[0], indexed_xy[1] - rest_xy[1]) > 0.012,
        details=f"rest_xy={rest_xy}, indexed_xy={indexed_xy}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
