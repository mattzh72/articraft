from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PROXIMAL_LENGTH = 0.045
MIDDLE_LENGTH = 0.031
DISTAL_LENGTH = 0.023

BASE_OUTER_RADIUS = 0.0082
BASE_EAR_WIDTH = 0.0090
BASE_GAP_WIDTH = 0.0140
PIP_OUTER_RADIUS = 0.0069
PIP_EAR_WIDTH = 0.0075
PIP_GAP_WIDTH = 0.0120

DIP_OUTER_RADIUS = 0.0059
DIP_EAR_WIDTH = 0.0065
DIP_GAP_WIDTH = 0.0100

KNUCKLE_AXIS = (0.0, 1.0, 0.0)


def _y_cylinder(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("XZ").cylinder(length, radius)


def _fork_ears(
    *,
    outer_radius: float,
    ear_width: float,
    gap_width: float,
) -> cq.Workplane:
    y_offset = gap_width / 2.0 + ear_width / 2.0
    left_ear = _y_cylinder(ear_width, outer_radius).translate((0.0, -y_offset, 0.0))
    right_ear = _y_cylinder(ear_width, outer_radius).translate((0.0, y_offset, 0.0))
    return left_ear.union(right_ear)


def _fork_cheeks(
    *,
    ear_width: float,
    gap_width: float,
    support_len: float,
    support_height: float,
    x_start: float,
    z_start: float,
) -> cq.Workplane:
    y_offset = gap_width / 2.0 + ear_width / 2.0
    left_cheek = (
        cq.Workplane("XY")
        .box(support_len, ear_width + 0.0010, support_height, centered=(False, True, False))
        .translate((x_start, -y_offset, z_start))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(support_len, ear_width + 0.0010, support_height, centered=(False, True, False))
        .translate((x_start, y_offset, z_start))
    )
    return left_cheek.union(right_cheek)


def _underside_cutter(x_min: float, x_max: float, width: float, flat_z: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x_max - x_min, width, 0.080, centered=(False, True, False))
        .translate((x_min, 0.0, flat_z - 0.080))
    )


def _ellipse_section(width: float, height: float, x_pos: float, z_pos: float) -> cq.Sketch:
    return cq.Sketch().ellipse(width / 2.0, height / 2.0).moved(x=x_pos, z=z_pos)


def _lofted_digit_body(sections: list[tuple[float, float, float, float]]) -> cq.Workplane:
    return cq.Workplane("YZ").placeSketch(*[_ellipse_section(*section) for section in sections]).loft()


def _make_base_block_shape() -> cq.Workplane:
    rear_block = (
        cq.Workplane("XY")
        .box(0.028, 0.038, 0.018, centered=(False, True, False))
        .translate((-0.034, 0.0, -0.018))
    )
    dorsal_round = _y_cylinder(0.034, 0.0108).translate((-0.021, 0.0, -0.0045))
    front_web = (
        cq.Workplane("XY")
        .box(0.014, 0.024, 0.014, centered=(False, True, False))
        .translate((-0.014, 0.0, -0.014))
    )
    knuckle_bulb = _y_cylinder(0.022, BASE_OUTER_RADIUS * 1.04).translate((-BASE_OUTER_RADIUS * 1.04, 0.0, 0.0))
    return rear_block.union(dorsal_round).union(front_web).union(knuckle_bulb).cut(
        _underside_cutter(-0.036, 0.004, 0.050, -0.018)
    )


def _make_segment_shape(
    *,
    length: float,
    root_radius: float,
    root_width: float,
    distal_radius: float | None = None,
    distal_width: float | None = None,
    widths: tuple[float, float, float],
    heights: tuple[float, float, float],
    z_offsets: tuple[float, float, float],
    palm_flat_z: float,
    tip_radius: float | None = None,
) -> cq.Workplane:
    def block(x_start: float, x_len: float, width: float, height: float, z_center: float) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(x_len, width, height, centered=(False, True, False))
            .translate((x_start, 0.0, z_center - height / 2.0))
        )

    root_bridge_len = max(root_radius * 1.45, 0.010)
    root_bridge_width = max(widths[0] * 0.82, root_width * 0.72)
    root_bridge_height = max(heights[0] * 0.92, root_radius * 1.35)
    root_bulb = _y_cylinder(root_width, root_radius).translate((root_radius, 0.0, 0.0))
    root_bridge = block(
        0.0,
        root_bridge_len,
        root_bridge_width,
        root_bridge_height,
        max(z_offsets[0] * 0.82, palm_flat_z + root_bridge_height / 2.0),
    )

    mid1_start = root_bridge_len * 0.55
    mid1_len = max(length * 0.24, 0.008)
    mid2_start = mid1_start + mid1_len * 0.70
    mid2_len = max(length * 0.22, 0.007)

    if distal_radius is not None:
        distal_anchor_x = length - distal_radius
        final_start = mid2_start + mid2_len * 0.72
        final_len = max(distal_anchor_x - final_start + distal_radius * 0.55, 0.0065)
    else:
        distal_anchor_x = length - (tip_radius or heights[-1] * 0.50)
        final_start = mid2_start + mid2_len * 0.72
        final_len = max(distal_anchor_x - final_start, 0.0055)

    main_body = (
        block(mid1_start, mid1_len, widths[0], heights[0], z_offsets[0])
        .union(block(mid2_start, mid2_len, widths[1], heights[1], z_offsets[1]))
        .union(block(final_start, final_len, widths[2], heights[2], z_offsets[2]))
    )

    segment = root_bulb.union(root_bridge).union(main_body)

    if distal_radius is not None:
        distal_width_value = distal_width or widths[2] * 0.92
        distal_web_len = max(distal_radius * 1.10, 0.0060)
        distal_web = block(
            distal_anchor_x - distal_web_len * 0.95,
            distal_web_len,
            max(widths[2] * 0.88, distal_width_value * 0.78),
            max(heights[2] * 0.96, distal_radius * 1.15),
            z_offsets[2],
        )
        distal_bulb = _y_cylinder(distal_width_value, distal_radius).translate((distal_anchor_x, 0.0, 0.0))
        segment = segment.union(distal_web).union(distal_bulb)
    else:
        tip_radius_value = tip_radius or heights[-1] * 0.52
        tip_neck = block(
            distal_anchor_x - tip_radius_value * 1.15,
            max(tip_radius_value * 1.15, 0.0048),
            widths[2] * 0.86,
            max(heights[2] * 0.95, tip_radius_value * 1.05),
            z_offsets[2],
        )
        tip = cq.Workplane("XY").sphere(tip_radius_value).translate((distal_anchor_x, 0.0, z_offsets[2] - 0.0003))
        segment = segment.union(tip_neck).union(tip)

    return segment.cut(
        _underside_cutter(
            -0.001,
            length + 0.010,
            max(root_width + 0.018, widths[0] + 0.014),
            palm_flat_z,
        )
    )


def _extents(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple(aabb[1][i] - aabb[0][i] for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_knuckle_finger")

    base_finish = model.material("base_finish", rgba=(0.17, 0.18, 0.20, 1.0))
    segment_finish = model.material("segment_finish", rgba=(0.76, 0.79, 0.82, 1.0))

    base_block = model.part("base_block")
    base_block.visual(
        mesh_from_cadquery(_make_base_block_shape(), "base_block"),
        material=base_finish,
        name="base_shell",
    )

    proximal_segment = model.part("proximal_segment")
    proximal_segment.visual(
        mesh_from_cadquery(
            _make_segment_shape(
                length=PROXIMAL_LENGTH,
                root_radius=BASE_OUTER_RADIUS * 0.95,
                root_width=BASE_GAP_WIDTH * 0.90,
                distal_radius=PIP_OUTER_RADIUS,
                distal_width=PIP_GAP_WIDTH * 0.90,
                widths=(0.0230, 0.0210, 0.0185),
                heights=(0.0175, 0.0158, 0.0136),
                z_offsets=(0.0020, 0.0028, 0.0026),
                palm_flat_z=-0.0060,
            ),
            "proximal_segment",
        ),
        material=segment_finish,
        name="proximal_shell",
    )

    middle_segment = model.part("middle_segment")
    middle_segment.visual(
        mesh_from_cadquery(
            _make_segment_shape(
                length=MIDDLE_LENGTH,
                root_radius=PIP_OUTER_RADIUS * 0.96,
                root_width=PIP_GAP_WIDTH * 0.90,
                distal_radius=DIP_OUTER_RADIUS,
                distal_width=DIP_GAP_WIDTH * 0.90,
                widths=(0.0190, 0.0170, 0.0148),
                heights=(0.0148, 0.0132, 0.0116),
                z_offsets=(0.0018, 0.0022, 0.0020),
                palm_flat_z=-0.0052,
            ),
            "middle_segment",
        ),
        material=segment_finish,
        name="middle_shell",
    )

    distal_segment = model.part("distal_segment")
    distal_segment.visual(
        mesh_from_cadquery(
            _make_segment_shape(
                length=DISTAL_LENGTH,
                root_radius=DIP_OUTER_RADIUS * 0.96,
                root_width=DIP_GAP_WIDTH * 0.90,
                widths=(0.0150, 0.0132, 0.0112),
                heights=(0.0118, 0.0103, 0.0089),
                z_offsets=(0.0015, 0.0017, 0.0014),
                palm_flat_z=-0.0038,
                tip_radius=0.0058,
            ),
            "distal_segment",
        ),
        material=segment_finish,
        name="distal_shell",
    )

    model.articulation(
        "base_knuckle",
        ArticulationType.REVOLUTE,
        parent=base_block,
        child=proximal_segment,
        origin=Origin(),
        axis=KNUCKLE_AXIS,
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.35),
    )
    model.articulation(
        "middle_knuckle",
        ArticulationType.REVOLUTE,
        parent=proximal_segment,
        child=middle_segment,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=KNUCKLE_AXIS,
        motion_limits=MotionLimits(effort=3.2, velocity=3.2, lower=0.0, upper=1.55),
    )
    model.articulation(
        "distal_knuckle",
        ArticulationType.REVOLUTE,
        parent=middle_segment,
        child=distal_segment,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=KNUCKLE_AXIS,
        motion_limits=MotionLimits(effort=2.4, velocity=3.4, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_block = object_model.get_part("base_block")
    proximal_segment = object_model.get_part("proximal_segment")
    middle_segment = object_model.get_part("middle_segment")
    distal_segment = object_model.get_part("distal_segment")

    base_knuckle = object_model.get_articulation("base_knuckle")
    middle_knuckle = object_model.get_articulation("middle_knuckle")
    distal_knuckle = object_model.get_articulation("distal_knuckle")

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

    ctx.expect_contact(
        base_block,
        proximal_segment,
        contact_tol=6e-4,
        name="base block supports proximal segment at the broad knuckle",
    )
    ctx.expect_contact(
        proximal_segment,
        middle_segment,
        contact_tol=6e-4,
        name="proximal and middle segments share a seated pip knuckle",
    )
    ctx.expect_contact(
        middle_segment,
        distal_segment,
        contact_tol=6e-4,
        name="middle and distal segments share a seated dip knuckle",
    )

    axes_match = (
        base_knuckle.axis == KNUCKLE_AXIS
        and middle_knuckle.axis == KNUCKLE_AXIS
        and distal_knuckle.axis == KNUCKLE_AXIS
    )
    ctx.check(
        "three revolute joints keep parallel knuckle axes",
        axes_match,
        details=(
            f"axes were {base_knuckle.axis}, {middle_knuckle.axis}, and {distal_knuckle.axis}; "
            f"expected all to equal {KNUCKLE_AXIS}"
        ),
    )

    prox_aabb = ctx.part_world_aabb(proximal_segment)
    mid_aabb = ctx.part_world_aabb(middle_segment)
    dist_aabb = ctx.part_world_aabb(distal_segment)
    if prox_aabb is not None and mid_aabb is not None and dist_aabb is not None:
        prox_size = _extents(prox_aabb)
        mid_size = _extents(mid_aabb)
        dist_size = _extents(dist_aabb)
        ctx.check(
            "digit segments decrease in size toward the tip",
            prox_size[0] > mid_size[0] > dist_size[0]
            and prox_size[1] > mid_size[1] > dist_size[1]
            and prox_size[2] > mid_size[2] > dist_size[2],
            details=(
                f"segment extents were proximal={prox_size}, middle={mid_size}, distal={dist_size}"
            ),
        )

    with ctx.pose():
        neutral_distal_origin = ctx.part_world_position(distal_segment)
    with ctx.pose(base_knuckle=1.0, middle_knuckle=1.18, distal_knuckle=0.78):
        curled_distal_origin = ctx.part_world_position(distal_segment)

    if neutral_distal_origin is not None and curled_distal_origin is not None:
        ctx.check(
            "positive joint motion curls the finger palmward",
            curled_distal_origin[0] < neutral_distal_origin[0] - 0.010
            and curled_distal_origin[2] < neutral_distal_origin[2] - 0.008,
            details=(
                f"distal origin moved from {neutral_distal_origin} to {curled_distal_origin}; "
                "expected a shorter x reach and lower z position in the flexed pose"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
