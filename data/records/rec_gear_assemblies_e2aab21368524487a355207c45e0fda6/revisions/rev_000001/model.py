from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FRAME_PART = "side_frame"
SHAFT_PARTS = (
    "input_shaft",
    "compound_shaft_a",
    "compound_shaft_b",
    "output_shaft",
)
JOINT_NAMES = (
    "frame_to_input_shaft",
    "frame_to_compound_shaft_a",
    "frame_to_compound_shaft_b",
    "frame_to_output_shaft",
)

PLATE_T = 0.012
PLATE_W = 0.340
PLATE_H = 0.200
FRONT_BOSS_T = 0.008
REAR_BOSS_T = 0.005
BOSS_R = 0.019
SHAFT_R = 0.006
JOURNAL_R = 0.0048
SUPPORT_BORE_R = 0.0072
SHAFT_D = SHAFT_R * 2.0
COLLAR_R = 0.0115
COLLAR_T = 0.004
SPACER_R = 0.010
SPACER_T = 0.004

GEAR_MODULE = 0.003
MESH_GAP = 0.0035
W_SMALL = 0.010
W_LARGE = 0.014

G1_TEETH = 20
G2A_TEETH = 34
G2B_TEETH = 16
G3A_TEETH = 30
G3B_TEETH = 18
G4_TEETH = 26

PLANE1_X = 0.040
PLANE2_X = 0.061
PLANE3_X = 0.083
OUTBOARD_END_X = 0.106


def pitch_distance(teeth_a: int, teeth_b: int) -> float:
    return GEAR_MODULE * (teeth_a + teeth_b) / 2.0 + MESH_GAP


SHAFT_Y = (
    -0.5 * (pitch_distance(G1_TEETH, G2A_TEETH) + pitch_distance(G2B_TEETH, G3A_TEETH) + pitch_distance(G3B_TEETH, G4_TEETH)),
    0.0,
    0.0,
    0.0,
)
SHAFT_Y = (
    SHAFT_Y[0],
    SHAFT_Y[0] + pitch_distance(G1_TEETH, G2A_TEETH),
    SHAFT_Y[0] + pitch_distance(G1_TEETH, G2A_TEETH) + pitch_distance(G2B_TEETH, G3A_TEETH),
    SHAFT_Y[0]
    + pitch_distance(G1_TEETH, G2A_TEETH)
    + pitch_distance(G2B_TEETH, G3A_TEETH)
    + pitch_distance(G3B_TEETH, G4_TEETH),
)

REAR_COLLAR_X = -PLATE_T / 2.0 - REAR_BOSS_T - COLLAR_T / 2.0
FRONT_COLLAR_X = PLATE_T / 2.0 + FRONT_BOSS_T + COLLAR_T / 2.0
SHAFT_CORE_START_X = -PLATE_T / 2.0 - REAR_BOSS_T
SHAFT_CORE_END_X = FRONT_COLLAR_X + COLLAR_T / 2.0
SHAFT_CORE_LENGTH = SHAFT_CORE_END_X - SHAFT_CORE_START_X
SHAFT_CORE_CENTER_X = 0.5 * (SHAFT_CORE_START_X + SHAFT_CORE_END_X)
OUTBOARD_SEAT_START_X = SHAFT_CORE_END_X
OUTBOARD_SEAT_LENGTH = OUTBOARD_END_X - OUTBOARD_SEAT_START_X
OUTBOARD_SEAT_CENTER_X = 0.5 * (OUTBOARD_SEAT_START_X + OUTBOARD_END_X)


def _cylinder_x(radius: float, length: float):
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def add_cylinder_x(part, *, radius: float, length: float, x_center: float, material, name: str) -> None:
    geom, base_origin = _cylinder_x(radius, length)
    part.visual(
        geom,
        origin=Origin(
            xyz=(x_center, 0.0, 0.0),
            rpy=base_origin.rpy,
        ),
        material=material,
        name=name,
    )


def build_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(PLATE_T, PLATE_W, PLATE_H)
    frame = frame.edges("|X").fillet(0.014)

    foot = cq.Workplane("XY").box(0.060, 0.150, 0.018).translate((-0.036, 0.0, -PLATE_H / 2.0 - 0.009))
    frame = frame.union(foot)

    for y_pos in SHAFT_Y:
        front_boss = cq.Workplane("YZ").circle(BOSS_R).extrude(FRONT_BOSS_T).translate((PLATE_T / 2.0, y_pos, 0.0))
        rear_boss = cq.Workplane("YZ").circle(BOSS_R).extrude(REAR_BOSS_T).translate(
            (-PLATE_T / 2.0 - REAR_BOSS_T, y_pos, 0.0)
        )
        frame = frame.union(front_boss).union(rear_boss)

        hole_depth = PLATE_T + FRONT_BOSS_T + REAR_BOSS_T + 0.002
        hole = cq.Workplane("YZ").circle(SUPPORT_BORE_R).extrude(hole_depth).translate(
            (-PLATE_T / 2.0 - REAR_BOSS_T - 0.001, y_pos, 0.0)
        )
        frame = frame.cut(hole)

    for z_pos in (-0.055, 0.055):
        slot = (
            cq.Workplane("XY")
            .box(0.032, 0.215, 0.028)
            .edges("|X")
            .fillet(0.010)
            .translate((0.0, 0.0, z_pos))
        )
        frame = frame.cut(slot)

    return frame.clean()


def build_gear_mesh(*, teeth: int, width: float, x_center: float, phase_deg: float, name: str):
    pitch_radius = GEAR_MODULE * teeth / 2.0
    outer_radius = pitch_radius + 0.95 * GEAR_MODULE
    root_radius = max(pitch_radius - 1.10 * GEAR_MODULE, SHAFT_R * 1.55)
    tooth_pitch = 2.0 * math.pi / teeth
    flank_inner = 0.34 * tooth_pitch
    flank_outer = 0.12 * tooth_pitch

    pts: list[tuple[float, float]] = []
    for tooth_index in range(teeth):
        center_angle = tooth_index * tooth_pitch
        pts.extend(
            [
                (
                    root_radius * math.cos(center_angle - flank_inner),
                    root_radius * math.sin(center_angle - flank_inner),
                ),
                (
                    outer_radius * math.cos(center_angle - flank_outer),
                    outer_radius * math.sin(center_angle - flank_outer),
                ),
                (
                    outer_radius * math.cos(center_angle + flank_outer),
                    outer_radius * math.sin(center_angle + flank_outer),
                ),
                (
                    root_radius * math.cos(center_angle + flank_inner),
                    root_radius * math.sin(center_angle + flank_inner),
                ),
            ]
        )

    body = (
        cq.Workplane("XY")
        .polyline(pts)
        .close()
        .extrude(width)
        .faces(">Z")
        .workplane()
        .circle(JOURNAL_R * 0.8)
        .cutThruAll()
        .val()
    )
    oriented = (
        cq.Workplane(obj=body)
        .translate((0.0, 0.0, -width / 2.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), phase_deg)
        .translate((x_center, 0.0, 0.0))
        .val()
    )
    return mesh_from_cadquery(oriented, name)


def add_shaft_part(model, *, name: str, gears: list[dict], steel_mat, brass_mat, dark_steel_mat):
    part = model.part(name)

    add_cylinder_x(
        part,
        radius=JOURNAL_R,
        length=SHAFT_CORE_LENGTH,
        x_center=SHAFT_CORE_CENTER_X,
        material=steel_mat,
        name="shaft_core",
    )
    add_cylinder_x(
        part,
        radius=SHAFT_R,
        length=OUTBOARD_SEAT_LENGTH,
        x_center=OUTBOARD_SEAT_CENTER_X,
        material=steel_mat,
        name="shaft_seat",
    )
    add_cylinder_x(
        part,
        radius=COLLAR_R,
        length=COLLAR_T,
        x_center=REAR_COLLAR_X,
        material=dark_steel_mat,
        name="rear_collar",
    )
    add_cylinder_x(
        part,
        radius=COLLAR_R,
        length=COLLAR_T,
        x_center=FRONT_COLLAR_X,
        material=dark_steel_mat,
        name="front_collar",
    )

    for gear_spec in gears:
        hub_radius = gear_spec["hub_radius"]
        hub_length = gear_spec["width"] + gear_spec.get("hub_extra", 0.004)
        add_cylinder_x(
            part,
            radius=hub_radius,
            length=hub_length,
            x_center=gear_spec["x_center"],
            material=dark_steel_mat,
            name=f"{gear_spec['name']}_hub",
        )
        part.visual(
            build_gear_mesh(
                teeth=gear_spec["teeth"],
                width=gear_spec["width"],
                x_center=gear_spec["x_center"],
                phase_deg=gear_spec.get("phase_deg", 0.0),
                name=gear_spec["name"],
            ),
            material=brass_mat if gear_spec.get("brass", False) else steel_mat,
            name=gear_spec["name"],
        )

    spacer_positions = []
    if len(gears) > 1:
        spacer_positions.append(0.5 * (gears[0]["x_center"] + gears[1]["x_center"]))
    spacer_positions.append(gears[-1]["x_center"] + gears[-1]["width"] / 2.0 + 0.008)

    for i, x_pos in enumerate(spacer_positions, start=1):
        add_cylinder_x(
            part,
            radius=SPACER_R,
            length=SPACER_T,
            x_center=x_pos,
            material=dark_steel_mat,
            name=f"spacer_{i}",
        )

    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compound_spur_train")

    frame_mat = model.material("frame_paint", rgba=(0.17, 0.20, 0.26, 1.0))
    steel_mat = model.material("steel", rgba=(0.74, 0.76, 0.79, 1.0))
    dark_steel_mat = model.material("dark_steel", rgba=(0.46, 0.49, 0.54, 1.0))
    brass_mat = model.material("brass", rgba=(0.73, 0.61, 0.30, 1.0))

    frame = model.part(FRAME_PART)
    frame.visual(mesh_from_cadquery(build_frame_shape(), "side_frame"), material=frame_mat, name="frame_shell")

    shaft_configs = {
        "input_shaft": [
            {
                "name": "input_gear",
                "teeth": G1_TEETH,
                "width": W_SMALL,
                "x_center": PLANE1_X,
                "phase_deg": 0.0,
                "hub_radius": 0.013,
            }
        ],
        "compound_shaft_a": [
            {
                "name": "stage1_gear",
                "teeth": G2A_TEETH,
                "width": W_LARGE,
                "x_center": PLANE1_X,
                "phase_deg": 180.0 / G2A_TEETH,
                "hub_radius": 0.016,
                "brass": True,
            },
            {
                "name": "stage1_pinion",
                "teeth": G2B_TEETH,
                "width": W_SMALL,
                "x_center": PLANE2_X,
                "phase_deg": 8.0,
                "hub_radius": 0.013,
            },
        ],
        "compound_shaft_b": [
            {
                "name": "stage2_gear",
                "teeth": G3A_TEETH,
                "width": W_LARGE,
                "x_center": PLANE2_X,
                "phase_deg": 180.0 / G3A_TEETH,
                "hub_radius": 0.015,
                "brass": True,
            },
            {
                "name": "stage2_pinion",
                "teeth": G3B_TEETH,
                "width": W_SMALL,
                "x_center": PLANE3_X,
                "phase_deg": 12.0,
                "hub_radius": 0.013,
            },
        ],
        "output_shaft": [
            {
                "name": "output_gear",
                "teeth": G4_TEETH,
                "width": W_LARGE,
                "x_center": PLANE3_X,
                "phase_deg": 180.0 / G4_TEETH,
                "hub_radius": 0.015,
                "brass": True,
            }
        ],
    }

    parts = {
        shaft_name: add_shaft_part(
            model,
            name=shaft_name,
            gears=shaft_configs[shaft_name],
            steel_mat=steel_mat,
            brass_mat=brass_mat,
            dark_steel_mat=dark_steel_mat,
        )
        for shaft_name in SHAFT_PARTS
    }

    for joint_name, shaft_name, y_pos in zip(JOINT_NAMES, SHAFT_PARTS, SHAFT_Y):
        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=parts[shaft_name],
            origin=Origin(xyz=(0.0, y_pos, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=6.0,
                lower=-2.0 * math.pi,
                upper=2.0 * math.pi,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part(FRAME_PART)
    input_shaft = object_model.get_part("input_shaft")
    shaft_a = object_model.get_part("compound_shaft_a")
    shaft_b = object_model.get_part("compound_shaft_b")
    output_shaft = object_model.get_part("output_shaft")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    joints = [object_model.get_articulation(name) for name in JOINT_NAMES]
    for shaft, joint in zip((input_shaft, shaft_a, shaft_b, output_shaft), joints):
        ctx.expect_contact(shaft, frame, name=f"{shaft.name}_supported_by_frame")
        axis_ok = tuple(round(v, 6) for v in joint.axis) == (1.0, 0.0, 0.0)
        limits = joint.motion_limits
        limit_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper
        )
        ctx.check(f"{joint.name}_axis_is_shaft_centerline", axis_ok, details=f"axis={joint.axis}")
        ctx.check(
            f"{joint.name}_has_bidirectional_rotation_limits",
            limit_ok,
            details=f"limits={limits}",
        )

    ctx.expect_gap(
        input_shaft,
        frame,
        axis="x",
        positive_elem="input_gear",
        min_gap=0.016,
        name="input_gear_clears_frame_supports",
    )
    ctx.expect_gap(
        shaft_a,
        frame,
        axis="x",
        positive_elem="stage1_gear",
        min_gap=0.012,
        name="stage1_gear_clears_frame_supports",
    )
    ctx.expect_gap(
        shaft_a,
        frame,
        axis="x",
        positive_elem="stage1_pinion",
        min_gap=0.028,
        name="stage1_pinion_clears_frame_supports",
    )
    ctx.expect_gap(
        shaft_b,
        frame,
        axis="x",
        positive_elem="stage2_gear",
        min_gap=0.028,
        name="stage2_gear_clears_frame_supports",
    )
    ctx.expect_gap(
        shaft_b,
        frame,
        axis="x",
        positive_elem="stage2_pinion",
        min_gap=0.050,
        name="stage2_pinion_clears_frame_supports",
    )
    ctx.expect_gap(
        output_shaft,
        frame,
        axis="x",
        positive_elem="output_gear",
        min_gap=0.050,
        name="output_gear_clears_frame_supports",
    )

    ctx.expect_overlap(
        input_shaft,
        shaft_a,
        axes="xz",
        elem_a="input_gear",
        elem_b="stage1_gear",
        min_overlap=0.009,
        name="stage1_meshes_in_shared_plane",
    )
    ctx.expect_overlap(
        shaft_a,
        shaft_b,
        axes="xz",
        elem_a="stage1_pinion",
        elem_b="stage2_gear",
        min_overlap=0.009,
        name="stage2_meshes_in_shared_plane",
    )
    ctx.expect_overlap(
        shaft_b,
        output_shaft,
        axes="xz",
        elem_a="stage2_pinion",
        elem_b="output_gear",
        min_overlap=0.009,
        name="stage3_meshes_in_shared_plane",
    )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=20,
        ignore_adjacent=True,
        name="gear_train_clear_in_sampled_poses",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
