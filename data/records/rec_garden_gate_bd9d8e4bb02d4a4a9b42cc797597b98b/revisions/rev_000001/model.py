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
    CylinderGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _merge_meshes(*geometries) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_latch_mesh() -> object:
    ring = TorusGeometry(
        radius=0.045,
        tube=0.006,
        radial_segments=18,
        tubular_segments=42,
    ).rotate_x(math.pi / 2.0)
    ring.translate(0.0, 0.005, -0.055)

    stem = BoxGeometry((0.010, 0.010, 0.022)).translate(0.0, 0.005, -0.014)
    pivot = CylinderGeometry(radius=0.008, height=0.010, radial_segments=18).rotate_x(math.pi / 2.0)
    pivot.translate(0.0, 0.005, 0.0)

    return _save_mesh("garden_gate_ring_latch", _merge_meshes(ring, stem, pivot))


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wrought_iron_garden_gate")

    stone = model.material("stone", rgba=(0.74, 0.71, 0.66, 1.0))
    stone_shadow = model.material("stone_shadow", rgba=(0.62, 0.59, 0.54, 1.0))
    wrought_iron = model.material("wrought_iron", rgba=(0.14, 0.15, 0.16, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.09, 0.10, 0.11, 1.0))

    pier_width = 0.38
    pier_depth = 0.34
    pier_height = 2.05
    cap_height = 0.10
    opening_width = 0.835
    gate_width = 0.80
    leaf_thickness = 0.020
    leaf_plane_y = 0.145
    bottom_clearance = 0.060
    shoulder_height = 1.65
    arch_rise = 0.22
    crest_height = shoulder_height + arch_rise

    left_pier_face_x = -0.015
    right_pier_face_x = left_pier_face_x + opening_width
    left_pier_center_x = left_pier_face_x - pier_width * 0.5
    right_pier_center_x = right_pier_face_x + pier_width * 0.5
    outer_left_x = left_pier_center_x - pier_width * 0.5
    outer_right_x = right_pier_center_x + pier_width * 0.5

    masonry_frame = model.part("masonry_frame")
    masonry_frame.visual(
        Box((outer_right_x - outer_left_x, pier_depth, 0.08)),
        origin=Origin(xyz=((outer_left_x + outer_right_x) * 0.5, 0.0, -0.04)),
        material=stone_shadow,
        name="foundation_strip",
    )
    masonry_frame.visual(
        Box((pier_width, pier_depth, pier_height)),
        origin=Origin(xyz=(left_pier_center_x, 0.0, pier_height * 0.5)),
        material=stone,
        name="left_pier",
    )
    masonry_frame.visual(
        Box((pier_width + 0.08, pier_depth + 0.08, cap_height)),
        origin=Origin(xyz=(left_pier_center_x, 0.0, pier_height + cap_height * 0.5)),
        material=stone_shadow,
        name="left_cap",
    )
    masonry_frame.visual(
        Box((pier_width, pier_depth, pier_height)),
        origin=Origin(xyz=(right_pier_center_x, 0.0, pier_height * 0.5)),
        material=stone,
        name="right_pier",
    )
    masonry_frame.visual(
        Box((pier_width + 0.08, pier_depth + 0.08, cap_height)),
        origin=Origin(xyz=(right_pier_center_x, 0.0, pier_height + cap_height * 0.5)),
        material=stone_shadow,
        name="right_cap",
    )

    for name, z_center in (
        ("hinge_pintle_low", 0.18),
        ("hinge_pintle_mid", 0.92),
        ("hinge_pintle_high", 1.62),
    ):
        masonry_frame.visual(
            Cylinder(radius=0.008, length=0.11),
            origin=Origin(xyz=(-0.008, leaf_plane_y, z_center)),
            material=dark_iron,
            name=name,
        )

    masonry_frame.visual(
        Box((0.050, 0.020, 0.060)),
        origin=Origin(xyz=(-0.115, 0.160, 1.79)),
        material=wrought_iron,
        name="closer_post_plate",
    )
    masonry_frame.visual(
        Cylinder(radius=0.009, length=0.012),
        origin=Origin(
            xyz=(-0.115, 0.176, 1.79),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="closer_post_bracket",
    )
    masonry_frame.visual(
        Box((0.020, 0.030, 0.140)),
        origin=Origin(xyz=(right_pier_face_x + 0.010, leaf_plane_y + 0.011, 1.00)),
        material=wrought_iron,
        name="right_strike_plate",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.030, leaf_thickness, shoulder_height)),
        origin=Origin(xyz=(0.015, 0.0, shoulder_height * 0.5)),
        material=wrought_iron,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((0.030, leaf_thickness, shoulder_height)),
        origin=Origin(xyz=(gate_width - 0.015, 0.0, shoulder_height * 0.5)),
        material=wrought_iron,
        name="meeting_stile",
    )
    gate_leaf.visual(
        Box((gate_width, leaf_thickness, 0.030)),
        origin=Origin(xyz=(gate_width * 0.5, 0.0, 0.015)),
        material=wrought_iron,
        name="bottom_rail",
    )

    arch_mesh = _save_mesh(
        "garden_gate_arch_top",
        tube_from_spline_points(
            [
                (0.015, 0.0, shoulder_height),
                (0.18, 0.0, shoulder_height + 0.08),
                (0.40, 0.0, crest_height),
                (0.62, 0.0, shoulder_height + 0.08),
                (gate_width - 0.015, 0.0, shoulder_height),
            ],
            radius=0.015,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    gate_leaf.visual(arch_mesh, material=wrought_iron, name="arch_top")

    for name, z_center in (
        ("hinge_tab_low", 0.18),
        ("hinge_tab_mid", 0.92),
        ("hinge_tab_high", 1.62),
    ):
        gate_leaf.visual(
            Box((0.050, leaf_thickness, 0.040)),
            origin=Origin(xyz=(0.025, 0.0, z_center)),
            material=wrought_iron,
            name=name,
        )

    half_span = gate_width * 0.5
    for index, x_pos in enumerate((0.12, 0.23, 0.34, 0.46, 0.57, 0.68)):
        arch_factor = 1.0 - ((x_pos - half_span) / (half_span - 0.03)) ** 2
        bar_top = shoulder_height + arch_rise * max(0.0, arch_factor) - 0.045
        bar_length = bar_top - 0.020
        gate_leaf.visual(
            Cylinder(radius=0.007, length=bar_length),
            origin=Origin(xyz=(x_pos, 0.0, 0.020 + bar_length * 0.5)),
            material=wrought_iron,
            name=f"vertical_bar_{index:02d}",
        )

    gate_leaf.visual(
        Box((0.060, 0.008, 0.060)),
        origin=Origin(xyz=(0.105, 0.014, 1.73)),
        material=wrought_iron,
        name="closer_leaf_plate",
    )
    gate_leaf.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(
            xyz=(0.105, 0.018, 1.73),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="closer_leaf_pivot",
    )
    gate_leaf.visual(
        Box((0.050, 0.008, 0.120)),
        origin=Origin(xyz=(0.758, 0.014, 1.01)),
        material=wrought_iron,
        name="latch_plate",
    )
    gate_leaf.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(
            xyz=(0.758, 0.018, 1.03),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="latch_pivot",
    )

    ring_latch = model.part("ring_latch")
    ring_latch.visual(
        _ring_latch_mesh(),
        material=dark_iron,
        name="ring_loop",
    )

    closer_primary_arm = model.part("closer_primary_arm")
    closer_primary_arm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="start_eye",
    )
    closer_primary_arm.visual(
        Box((0.120, 0.012, 0.014)),
        origin=Origin(xyz=(-0.060, 0.006, 0.0)),
        material=wrought_iron,
        name="arm_span",
    )
    closer_primary_arm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(-0.120, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="end_eye",
    )

    closer_secondary_arm = model.part("closer_secondary_arm")
    closer_secondary_arm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(0.0, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="start_eye",
    )
    closer_secondary_arm.visual(
        Box((0.100, 0.012, 0.014)),
        origin=Origin(xyz=(-0.050, 0.006, 0.0)),
        material=wrought_iron,
        name="arm_span",
    )
    closer_secondary_arm.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(
            xyz=(-0.100, 0.006, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_iron,
        name="end_eye",
    )

    model.articulation(
        "masonry_to_gate_leaf",
        ArticulationType.REVOLUTE,
        parent=masonry_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, leaf_plane_y, bottom_clearance)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.1,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "gate_leaf_to_ring_latch",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=ring_latch,
        origin=Origin(xyz=(0.758, 0.023, 1.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-1.0,
            upper=0.7,
        ),
    )
    model.articulation(
        "gate_leaf_to_closer_primary",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=closer_primary_arm,
        origin=Origin(xyz=(0.105, 0.025, 1.73)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.7,
            upper=0.7,
        ),
    )
    model.articulation(
        "closer_primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=closer_primary_arm,
        child=closer_secondary_arm,
        origin=Origin(xyz=(-0.120, 0.012, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.6,
        ),
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

    masonry_frame = object_model.get_part("masonry_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    ring_latch = object_model.get_part("ring_latch")
    closer_primary_arm = object_model.get_part("closer_primary_arm")
    closer_secondary_arm = object_model.get_part("closer_secondary_arm")

    gate_hinge = object_model.get_articulation("masonry_to_gate_leaf")
    latch_joint = object_model.get_articulation("gate_leaf_to_ring_latch")
    closer_primary_joint = object_model.get_articulation("gate_leaf_to_closer_primary")
    closer_secondary_joint = object_model.get_articulation("closer_primary_to_secondary")

    ctx.check("masonry frame exists", masonry_frame is not None)
    ctx.check("gate leaf exists", gate_leaf is not None)
    ctx.check("ring latch exists", ring_latch is not None)
    ctx.check("closer primary arm exists", closer_primary_arm is not None)
    ctx.check("closer secondary arm exists", closer_secondary_arm is not None)

    with ctx.pose(
        {
            gate_hinge: 0.0,
            latch_joint: 0.0,
            closer_primary_joint: 0.0,
            closer_secondary_joint: 0.0,
        }
    ):
        ctx.expect_contact(
            gate_leaf,
            masonry_frame,
            elem_a="hinge_tab_mid",
            elem_b="hinge_pintle_mid",
            name="leaf bears on hinge pintle",
        )
        ctx.expect_gap(
            gate_leaf,
            masonry_frame,
            axis="z",
            positive_elem="bottom_rail",
            negative_elem="foundation_strip",
            min_gap=0.055,
            max_gap=0.065,
            name="leaf clears the ground footing",
        )
        ctx.expect_gap(
            masonry_frame,
            gate_leaf,
            axis="x",
            positive_elem="right_strike_plate",
            negative_elem="meeting_stile",
            min_gap=0.0,
            max_gap=0.040,
            name="meeting edge sits near the strike plate",
        )
        ctx.expect_contact(
            closer_primary_arm,
            gate_leaf,
            elem_a="start_eye",
            elem_b="closer_leaf_pivot",
            name="closer primary arm is pinned to the leaf bracket",
        )
        ctx.expect_contact(
            closer_secondary_arm,
            closer_primary_arm,
            elem_a="start_eye",
            elem_b="end_eye",
            name="closer elbow is pinned between the arm links",
        )
        ctx.expect_contact(
            closer_secondary_arm,
            masonry_frame,
            elem_a="end_eye",
            elem_b="closer_post_bracket",
            name="closer arm reaches the pier bracket in the closed pose",
        )

        closed_meeting = _aabb_center(ctx.part_element_world_aabb(gate_leaf, elem="meeting_stile"))
        closed_ring = _aabb_center(ctx.part_element_world_aabb(ring_latch, elem="ring_loop"))
        closed_primary = _aabb_center(ctx.part_element_world_aabb(closer_primary_arm, elem="arm_span"))
        closed_secondary = _aabb_center(ctx.part_element_world_aabb(closer_secondary_arm, elem="arm_span"))

    with ctx.pose(
        {
            gate_hinge: gate_hinge.motion_limits.upper,
            latch_joint: 0.0,
            closer_primary_joint: 0.0,
            closer_secondary_joint: 0.0,
        }
    ):
        open_meeting = _aabb_center(ctx.part_element_world_aabb(gate_leaf, elem="meeting_stile"))

    ctx.check(
        "gate leaf opens toward positive y",
        closed_meeting is not None
        and open_meeting is not None
        and open_meeting[1] > closed_meeting[1] + 0.45,
        details=f"closed={closed_meeting}, open={open_meeting}",
    )

    with ctx.pose(
        {
            gate_hinge: 0.0,
            latch_joint: 0.55,
            closer_primary_joint: 0.0,
            closer_secondary_joint: 0.0,
        }
    ):
        rotated_ring = _aabb_center(ctx.part_element_world_aabb(ring_latch, elem="ring_loop"))

    ctx.check(
        "ring latch rotates on its pivot",
        closed_ring is not None
        and rotated_ring is not None
        and abs(rotated_ring[0] - closed_ring[0]) > 0.020,
        details=f"closed={closed_ring}, rotated={rotated_ring}",
    )

    with ctx.pose(
        {
            gate_hinge: 0.0,
            latch_joint: 0.0,
            closer_primary_joint: 0.45,
            closer_secondary_joint: 0.0,
        }
    ):
        swung_primary = _aabb_center(ctx.part_element_world_aabb(closer_primary_arm, elem="arm_span"))

    ctx.check(
        "closer primary arm rotates at the leaf-side pivot",
        closed_primary is not None
        and swung_primary is not None
        and abs(swung_primary[2] - closed_primary[2]) > 0.015,
        details=f"closed={closed_primary}, swung={swung_primary}",
    )

    with ctx.pose(
        {
            gate_hinge: 0.0,
            latch_joint: 0.0,
            closer_primary_joint: 0.0,
            closer_secondary_joint: -0.55,
        }
    ):
        swung_secondary = _aabb_center(ctx.part_element_world_aabb(closer_secondary_arm, elem="arm_span"))

    ctx.check(
        "closer secondary arm rotates at the elbow pivot",
        closed_secondary is not None
        and swung_secondary is not None
        and abs(swung_secondary[2] - closed_secondary[2]) > 0.015,
        details=f"closed={closed_secondary}, swung={swung_secondary}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
