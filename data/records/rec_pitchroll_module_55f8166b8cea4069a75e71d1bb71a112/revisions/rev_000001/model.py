from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_frame_shape() -> cq.Workplane:
    cheek_hole_radius = 0.0145
    cheek_thickness = 0.014

    left_cheek = cq.Workplane("YZ").rect(0.092, 0.104).extrude(cheek_thickness).translate((-0.059, 0.0, 0.0))
    left_cheek = left_cheek.cut(
        cq.Workplane("YZ").circle(cheek_hole_radius).extrude(cheek_thickness).translate((-0.059, 0.0, 0.0))
    )
    right_cheek = cq.Workplane("YZ").rect(0.092, 0.104).extrude(cheek_thickness).translate((0.045, 0.0, 0.0))
    right_cheek = right_cheek.cut(
        cq.Workplane("YZ").circle(cheek_hole_radius).extrude(cheek_thickness).translate((0.045, 0.0, 0.0))
    )

    frame = left_cheek.union(right_cheek)
    frame = frame.union(cq.Workplane("XY").box(0.118, 0.044, 0.016).translate((0.0, 0.0, -0.060)))
    frame = frame.union(cq.Workplane("XY").box(0.024, 0.076, 0.024).translate((-0.045, 0.0, -0.072)))
    frame = frame.union(cq.Workplane("XY").box(0.024, 0.076, 0.024).translate((0.045, 0.0, -0.072)))
    frame = frame.union(cq.Workplane("XY").box(0.118, 0.014, 0.012).translate((0.0, -0.039, 0.050)))
    return frame


def make_roll_cartridge_shape() -> cq.Workplane:
    cartridge = cq.Workplane("YZ").circle(0.032).extrude(0.070).translate((-0.035, 0.0, 0.0))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.036).extrude(0.006).translate((-0.041, 0.0, 0.0)))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.036).extrude(0.006).translate((0.035, 0.0, 0.0)))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.0145).extrude(0.017).translate((-0.058, 0.0, 0.0)))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.0145).extrude(0.017).translate((0.041, 0.0, 0.0)))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.020).extrude(0.004).translate((-0.062, 0.0, 0.0)))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.020).extrude(0.004).translate((0.058, 0.0, 0.0)))
    cartridge = cartridge.union(cq.Workplane("YZ").circle(0.018).extrude(0.010).translate((0.062, 0.0, 0.0)))
    return cartridge


def make_pitch_yoke_shape() -> cq.Workplane:
    yoke = cq.Workplane("YZ").circle(0.016).extrude(0.012).translate((0.0, 0.0, 0.0))
    yoke = yoke.union(cq.Workplane("XY").box(0.016, 0.056, 0.036).translate((0.008, 0.0, 0.0)))
    yoke = yoke.union(cq.Workplane("XY").box(0.034, 0.012, 0.044).translate((0.031, -0.028, 0.0)))
    yoke = yoke.union(cq.Workplane("XY").box(0.034, 0.012, 0.044).translate((0.031, 0.028, 0.0)))
    yoke = yoke.cut(cq.Workplane("XZ").circle(0.0075).extrude(0.080).translate((0.046, -0.040, 0.0)))
    return yoke


def make_output_face_shape() -> cq.Workplane:
    output_face = cq.Workplane("XZ").circle(0.0065).extrude(0.044).translate((0.0, -0.022, 0.0))
    output_face = output_face.union(cq.Workplane("XZ").circle(0.0075).extrude(0.012).translate((0.0, -0.034, 0.0)))
    output_face = output_face.union(cq.Workplane("XZ").circle(0.0075).extrude(0.012).translate((0.0, 0.022, 0.0)))
    output_face = output_face.union(cq.Workplane("XZ").circle(0.010).extrude(0.004).translate((0.0, -0.038, 0.0)))
    output_face = output_face.union(cq.Workplane("XZ").circle(0.010).extrude(0.004).translate((0.0, 0.034, 0.0)))
    output_face = output_face.union(cq.Workplane("XY").box(0.024, 0.028, 0.032).translate((0.016, 0.0, 0.0)))
    output_face = output_face.union(cq.Workplane("XY").box(0.034, 0.032, 0.042).translate((0.037, 0.0, 0.0)))
    output_face = output_face.union(cq.Workplane("XY").box(0.012, 0.072, 0.060).translate((0.060, 0.0, 0.0)))
    output_face = output_face.faces(">X").workplane().rect(0.054, 0.040).cutBlind(-0.0025)
    return output_face


def make_cheek_plate(hole_radius: float = 0.018) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .rect(0.100, 0.100)
        .circle(hole_radius)
        .extrude(0.014)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_spindle_pitch_roll_unit")

    model.material("frame_gray", rgba=(0.25, 0.27, 0.30, 1.0))
    model.material("cartridge_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    model.material("yoke_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("plate_silver", rgba=(0.80, 0.82, 0.84, 1.0))
    model.material("shaft_steel", rgba=(0.55, 0.58, 0.60, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(make_cheek_plate(), "left_cheek"),
        origin=Origin(xyz=(-0.059, 0.0, 0.0)),
        material="frame_gray",
        name="left_cheek",
    )
    frame.visual(
        mesh_from_cadquery(make_cheek_plate(), "right_cheek"),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material="frame_gray",
        name="right_cheek",
    )
    frame.visual(
        Box((0.118, 0.048, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material="frame_gray",
        name="base_bridge",
    )
    frame.visual(
        Box((0.024, 0.072, 0.024)),
        origin=Origin(xyz=(-0.040, 0.0, -0.070)),
        material="frame_gray",
        name="left_web",
    )
    frame.visual(
        Box((0.024, 0.072, 0.024)),
        origin=Origin(xyz=(0.040, 0.0, -0.070)),
        material="frame_gray",
        name="right_web",
    )
    frame.visual(
        Box((0.118, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.036, 0.048)),
        material="frame_gray",
        name="rear_tie",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        Cylinder(radius=0.014, length=0.090),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="cartridge_gray",
        name="roll_spindle",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.032, length=0.060),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="cartridge_gray",
        name="roll_drum",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="cartridge_gray",
        name="rear_cap",
    )
    roll_cartridge.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.052, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="cartridge_gray",
        name="cartridge_nose",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="yoke_black",
        name="mount_hub",
    )
    pitch_yoke.visual(
        Box((0.018, 0.056, 0.032)),
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        material="yoke_black",
        name="yoke_bridge",
    )
    pitch_yoke.visual(
        Box((0.032, 0.012, 0.044)),
        origin=Origin(xyz=(0.042, 0.028, 0.0)),
        material="yoke_black",
        name="upper_arm",
    )
    pitch_yoke.visual(
        Box((0.032, 0.012, 0.044)),
        origin=Origin(xyz=(0.042, -0.028, 0.0)),
        material="yoke_black",
        name="lower_arm",
    )

    output_face = model.part("output_face")
    output_face.visual(
        Cylinder(radius=0.0065, length=0.044),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="shaft_steel",
        name="pitch_spindle",
    )
    output_face.visual(
        Box((0.018, 0.028, 0.028)),
        origin=Origin(xyz=(0.009, 0.0, 0.0)),
        material="shaft_steel",
        name="spindle_boss",
    )
    output_face.visual(
        Box((0.034, 0.032, 0.042)),
        origin=Origin(xyz=(0.029, 0.0, 0.0)),
        material="plate_silver",
        name="face_stem",
    )
    output_face.visual(
        Box((0.012, 0.072, 0.060)),
        origin=Origin(xyz=(0.052, 0.0, 0.0)),
        material="plate_silver",
        name="face_plate",
    )

    model.articulation(
        "frame_to_roll_cartridge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=roll_cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.0,
            lower=-1.75,
            upper=1.75,
        ),
    )
    model.articulation(
        "roll_cartridge_to_pitch_yoke",
        ArticulationType.FIXED,
        parent=roll_cartridge,
        child=pitch_yoke,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )
    model.articulation(
        "pitch_yoke_to_output_face",
        ArticulationType.REVOLUTE,
        parent=pitch_yoke,
        child=output_face,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.0,
            lower=-0.85,
            upper=0.65,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    roll_cartridge = object_model.get_part("roll_cartridge")
    pitch_yoke = object_model.get_part("pitch_yoke")
    output_face = object_model.get_part("output_face")
    roll_joint = object_model.get_articulation("frame_to_roll_cartridge")
    pitch_joint = object_model.get_articulation("pitch_yoke_to_output_face")

    def span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    def center_axis(
        aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int
    ) -> float:
        return 0.5 * (aabb[0][axis] + aabb[1][axis])

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        frame,
        roll_cartridge,
        elem_a="right_cheek",
        elem_b="cartridge_nose",
        reason="The roll cartridge journal is captured in the fork cheek bore; the zero-clearance bearing seat can register as slight mesh interference.",
    )
    ctx.allow_overlap(
        frame,
        roll_cartridge,
        elem_a="left_cheek",
        elem_b="rear_cap",
        reason="The opposite roll journal is likewise seated in the fork cheek bore, so the mesh-backed cheek and cylindrical bearing land are allowed a slight intended press-fit overlap.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(frame, roll_cartridge, name="frame_supports_roll_cartridge")
    ctx.expect_contact(
        roll_cartridge,
        pitch_yoke,
        elem_a="cartridge_nose",
        elem_b="mount_hub",
        name="roll_cartridge_carries_pitch_yoke",
    )
    ctx.expect_contact(
        pitch_yoke,
        output_face,
        elem_b="pitch_spindle",
        name="pitch_yoke_supports_output_face",
    )
    ctx.expect_overlap(
        frame,
        roll_cartridge,
        axes="yz",
        min_overlap=0.060,
        name="roll_cartridge_sits_within_frame_envelope",
    )

    ctx.check(
        "roll_axis_matches_longitudinal_spindle_axis",
        roll_joint.axis == (1.0, 0.0, 0.0),
        details=f"Expected roll axis (1, 0, 0), got {roll_joint.axis}",
    )
    ctx.check(
        "pitch_axis_matches_yoke_cross_axis",
        pitch_joint.axis == (0.0, -1.0, 0.0),
        details=f"Expected pitch axis (0, -1, 0), got {pitch_joint.axis}",
    )

    neutral_plate = ctx.part_element_world_aabb(output_face, elem="face_plate")
    with ctx.pose({pitch_joint: 0.45}):
        pitched_plate = ctx.part_element_world_aabb(output_face, elem="face_plate")
    ctx.check(
        "positive_pitch_raises_output_face",
        neutral_plate is not None
        and pitched_plate is not None
        and center_axis(pitched_plate, 2) > center_axis(neutral_plate, 2) + 0.006,
        details=(
            f"Neutral plate AABB={neutral_plate}, pitched plate AABB={pitched_plate}; "
            "expected positive pitch to raise the rectangular face along +Z."
        ),
    )

    with ctx.pose({roll_joint: pi / 2.0}):
        rolled_plate = ctx.part_element_world_aabb(output_face, elem="face_plate")
    ctx.check(
        "roll_reorients_output_face_in_its_plane",
        neutral_plate is not None
        and rolled_plate is not None
        and span(rolled_plate, 2) > span(neutral_plate, 2) + 0.008
        and span(rolled_plate, 1) < span(neutral_plate, 1) - 0.008,
        details=(
            f"Neutral plate AABB={neutral_plate}, rolled plate AABB={rolled_plate}; "
            "expected the face width/height orientation to swap as the cartridge rolls about +X."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
