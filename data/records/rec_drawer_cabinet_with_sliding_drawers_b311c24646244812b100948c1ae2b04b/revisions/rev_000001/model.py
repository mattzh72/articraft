from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


CABINET_WIDTH = 0.420
CABINET_DEPTH = 0.640
BODY_HEIGHT = 1.200
BODY_BASE_Z = 0.072
SIDE_THICKNESS = 0.012
BACK_THICKNESS = 0.010
BOTTOM_THICKNESS = 0.012

DRAWER_COUNT = 4
DRAWER_FACE_HEIGHT = 0.268
DRAWER_VERTICAL_GAP = 0.010
DRAWER_MARGIN_Z = 0.049
DRAWER_FRONT_WIDTH = 0.390
DRAWER_FRONT_THICKNESS = 0.018
DRAWER_BUCKET_WIDTH = 0.344
DRAWER_BUCKET_DEPTH = 0.570
DRAWER_BUCKET_HEIGHT = 0.214
DRAWER_SIDE_THICKNESS = 0.010
DRAWER_BOTTOM_THICKNESS = 0.010
DRAWER_BACK_THICKNESS = 0.012
DRAWER_TRAVEL = 0.350
DRAWER_CLOSED_Y = CABINET_DEPTH * 0.5 - DRAWER_FRONT_THICKNESS

OUTER_SLIDE_THICKNESS = 0.012
INNER_SLIDE_THICKNESS = 0.014
SLIDE_HEIGHT = 0.042
OUTER_SLIDE_LENGTH = 0.420
INNER_SLIDE_LENGTH = 0.380
OUTER_SLIDE_CENTER_Y = -0.080
INNER_SLIDE_CENTER_Y = -0.370

TOP_CAP_THICKNESS = 0.018
TOP_CAP_WIDTH = CABINET_WIDTH + 0.008
TOP_CAP_DEPTH = CABINET_DEPTH + 0.008
TOP_CAP_LIP_DROP = 0.028
TOP_CAP_LIP_THICKNESS = 0.012
TOP_CAP_HINGE_Y = -(CABINET_DEPTH * 0.5 + 0.004)


def _drawer_center_z(index: int) -> float:
    return (
        BODY_BASE_Z
        + DRAWER_MARGIN_Z
        + DRAWER_FACE_HEIGHT * 0.5
        + index * (DRAWER_FACE_HEIGHT + DRAWER_VERTICAL_GAP)
    )


def _add_caster_visuals(body, *, x: float, y: float, suffix: str, plate_material, fork_material, wheel_material) -> None:
    plate_w = 0.052
    plate_d = 0.040
    plate_t = 0.004
    stem_r = 0.008
    stem_len = 0.018
    fork_w = 0.032
    fork_d = 0.024
    fork_h = 0.032
    wheel_r = 0.028
    wheel_w = 0.020

    body.visual(
        Box((plate_w, plate_d, plate_t)),
        origin=Origin(xyz=(x, y, BODY_BASE_Z - plate_t * 0.5)),
        material=plate_material,
        name=f"caster_plate_{suffix}",
    )
    body.visual(
        Cylinder(radius=stem_r, length=stem_len),
        origin=Origin(xyz=(x, y, BODY_BASE_Z - 0.012)),
        material=fork_material,
        name=f"caster_stem_{suffix}",
    )
    body.visual(
        Box((fork_w, fork_d, fork_h)),
        origin=Origin(xyz=(x, y, 0.036)),
        material=fork_material,
        name=f"caster_fork_{suffix}",
    )
    body.visual(
        Cylinder(radius=wheel_r, length=wheel_w),
        origin=Origin(
            xyz=(x, y, wheel_r),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=wheel_material,
        name=f"caster_wheel_{suffix}",
    )


def _add_drawer(model: ArticulatedObject, body, *, drawer_number: int, z_center: float, face_material, bucket_material, handle_material) -> None:
    drawer = model.part(f"drawer_{drawer_number}")

    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, DRAWER_FRONT_THICKNESS, DRAWER_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, DRAWER_FRONT_THICKNESS * 0.5, 0.0)),
        material=face_material,
        name="front_face",
    )

    side_center_y = -0.281
    side_center_z = -0.027
    bucket_half_width = DRAWER_BUCKET_WIDTH * 0.5
    side_x = bucket_half_width - DRAWER_SIDE_THICKNESS * 0.5
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BUCKET_DEPTH, DRAWER_BUCKET_HEIGHT)),
        origin=Origin(xyz=(-side_x, side_center_y, side_center_z)),
        material=bucket_material,
        name="left_wall",
    )
    drawer.visual(
        Box((DRAWER_SIDE_THICKNESS, DRAWER_BUCKET_DEPTH, DRAWER_BUCKET_HEIGHT)),
        origin=Origin(xyz=(side_x, side_center_y, side_center_z)),
        material=bucket_material,
        name="right_wall",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BUCKET_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_BUCKET_DEPTH,
                DRAWER_BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(xyz=(0.0, -0.277, -0.134 + DRAWER_BOTTOM_THICKNESS * 0.5)),
        material=bucket_material,
        name="bottom_pan",
    )
    drawer.visual(
        Box(
            (
                DRAWER_BUCKET_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS,
                DRAWER_BACK_THICKNESS,
                DRAWER_BUCKET_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -DRAWER_BUCKET_DEPTH + DRAWER_BACK_THICKNESS * 0.5 + 0.004,
                side_center_z,
            )
        ),
        material=bucket_material,
        name="back_wall",
    )
    file_rail_x = bucket_half_width - DRAWER_SIDE_THICKNESS - 0.005
    drawer.visual(
        Box((0.010, DRAWER_BUCKET_DEPTH - 0.060, 0.010)),
        origin=Origin(xyz=(-file_rail_x, -0.288, 0.068)),
        material=handle_material,
        name="left_file_rail",
    )
    drawer.visual(
        Box((0.010, DRAWER_BUCKET_DEPTH - 0.060, 0.010)),
        origin=Origin(xyz=(file_rail_x, -0.288, 0.068)),
        material=handle_material,
        name="right_file_rail",
    )

    inner_slide_x = bucket_half_width + INNER_SLIDE_THICKNESS * 0.5
    slide_z = -0.006
    drawer.visual(
        Box((INNER_SLIDE_THICKNESS, INNER_SLIDE_LENGTH, SLIDE_HEIGHT)),
        origin=Origin(xyz=(-inner_slide_x, INNER_SLIDE_CENTER_Y, slide_z)),
        material=handle_material,
        name="inner_left_slide",
    )
    drawer.visual(
        Box((INNER_SLIDE_THICKNESS, INNER_SLIDE_LENGTH, SLIDE_HEIGHT)),
        origin=Origin(xyz=(inner_slide_x, INNER_SLIDE_CENTER_Y, slide_z)),
        material=handle_material,
        name="inner_right_slide",
    )

    post_offset_x = 0.082
    post_center_y = 0.030
    handle_center_z = 0.040
    drawer.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(-post_offset_x, post_center_y, handle_center_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=handle_material,
        name="left_handle_post",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(
            xyz=(post_offset_x, post_center_y, handle_center_z),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=handle_material,
        name="right_handle_post",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.196),
        origin=Origin(
            xyz=(0.0, 0.048, handle_center_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=handle_material,
        name="handle_bar",
    )

    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_WIDTH, DRAWER_BUCKET_DEPTH, DRAWER_FACE_HEIGHT)),
        mass=6.0,
        origin=Origin(xyz=(0.0, -0.170, -0.010)),
    )

    model.articulation(
        f"body_to_drawer_{drawer_number}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_CLOSED_Y, z_center)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="four_drawer_filing_cabinet")

    steel_body = model.material("steel_body", rgba=(0.44, 0.47, 0.50, 1.0))
    drawer_face = model.material("drawer_face", rgba=(0.55, 0.58, 0.62, 1.0))
    slide_steel = model.material("slide_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    caster_rubber = model.material("caster_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    model.meta["object_type"] = "filing_cabinet"

    body = model.part("body")
    wall_center_z = BODY_BASE_Z + BODY_HEIGHT * 0.5
    body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH * 0.5 - SIDE_THICKNESS * 0.5),
                0.0,
                wall_center_z,
            )
        ),
        material=steel_body,
        name="left_wall",
    )
    body.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH * 0.5 - SIDE_THICKNESS * 0.5,
                0.0,
                wall_center_z,
            )
        ),
        material=steel_body,
        name="right_wall",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, BODY_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH * 0.5 - BACK_THICKNESS * 0.5),
                wall_center_z,
            )
        ),
        material=steel_body,
        name="back_panel",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BODY_BASE_Z + BOTTOM_THICKNESS * 0.5,
            )
        ),
        material=steel_body,
        name="bottom_pan",
    )
    body.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.030, 0.022)),
        origin=Origin(
            xyz=(
                0.0,
                -(CABINET_DEPTH * 0.5 - 0.015),
                BODY_BASE_Z + BODY_HEIGHT - 0.011,
            )
        ),
        material=dark_steel,
        name="rear_top_crossmember",
    )

    outer_slide_x = CABINET_WIDTH * 0.5 - SIDE_THICKNESS - OUTER_SLIDE_THICKNESS * 0.5
    for drawer_index in range(DRAWER_COUNT):
        z_center = _drawer_center_z(drawer_index) - 0.006
        drawer_number = drawer_index + 1
        body.visual(
            Box((OUTER_SLIDE_THICKNESS, OUTER_SLIDE_LENGTH, SLIDE_HEIGHT)),
            origin=Origin(xyz=(-outer_slide_x, OUTER_SLIDE_CENTER_Y, z_center)),
            material=slide_steel,
            name=f"outer_left_slide_{drawer_number}",
        )
        body.visual(
            Box((OUTER_SLIDE_THICKNESS, OUTER_SLIDE_LENGTH, SLIDE_HEIGHT)),
            origin=Origin(xyz=(outer_slide_x, OUTER_SLIDE_CENTER_Y, z_center)),
            material=slide_steel,
            name=f"outer_right_slide_{drawer_number}",
        )

    caster_x = CABINET_WIDTH * 0.5 - 0.050
    caster_y = CABINET_DEPTH * 0.5 - 0.060
    _add_caster_visuals(
        body,
        x=-caster_x,
        y=-caster_y,
        suffix="rear_left",
        plate_material=dark_steel,
        fork_material=dark_steel,
        wheel_material=caster_rubber,
    )
    _add_caster_visuals(
        body,
        x=caster_x,
        y=-caster_y,
        suffix="rear_right",
        plate_material=dark_steel,
        fork_material=dark_steel,
        wheel_material=caster_rubber,
    )
    _add_caster_visuals(
        body,
        x=-caster_x,
        y=caster_y,
        suffix="front_left",
        plate_material=dark_steel,
        fork_material=dark_steel,
        wheel_material=caster_rubber,
    )
    _add_caster_visuals(
        body,
        x=caster_x,
        y=caster_y,
        suffix="front_right",
        plate_material=dark_steel,
        fork_material=dark_steel,
        wheel_material=caster_rubber,
    )

    body.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, BODY_BASE_Z + BODY_HEIGHT)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.0, (BODY_BASE_Z + BODY_HEIGHT) * 0.5)),
    )

    for drawer_index in range(DRAWER_COUNT):
        _add_drawer(
            model,
            body,
            drawer_number=drawer_index + 1,
            z_center=_drawer_center_z(drawer_index),
            face_material=drawer_face,
            bucket_material=steel_body,
            handle_material=slide_steel,
        )

    top_cap = model.part("top_cap")
    top_cap.visual(
        Box((TOP_CAP_WIDTH, TOP_CAP_DEPTH, TOP_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, TOP_CAP_DEPTH * 0.5, TOP_CAP_THICKNESS * 0.5)),
        material=drawer_face,
        name="cap_slab",
    )
    top_cap.visual(
        Box((TOP_CAP_WIDTH - 0.024, TOP_CAP_LIP_THICKNESS, TOP_CAP_LIP_DROP)),
        origin=Origin(
            xyz=(
                0.0,
                TOP_CAP_DEPTH - TOP_CAP_LIP_THICKNESS * 0.5,
                (TOP_CAP_THICKNESS - TOP_CAP_LIP_DROP) * 0.5,
            )
        ),
        material=drawer_face,
        name="front_return",
    )
    top_cap.visual(
        Box((TOP_CAP_LIP_THICKNESS, TOP_CAP_DEPTH - 0.044, TOP_CAP_LIP_DROP)),
        origin=Origin(
            xyz=(
                -(CABINET_WIDTH * 0.5 + TOP_CAP_LIP_THICKNESS * 0.5 + 0.001),
                0.022 + (TOP_CAP_DEPTH - 0.044) * 0.5,
                (TOP_CAP_THICKNESS - TOP_CAP_LIP_DROP) * 0.5,
            )
        ),
        material=drawer_face,
        name="left_return",
    )
    top_cap.visual(
        Box((TOP_CAP_LIP_THICKNESS, TOP_CAP_DEPTH - 0.044, TOP_CAP_LIP_DROP)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH * 0.5 + TOP_CAP_LIP_THICKNESS * 0.5 + 0.001,
                0.022 + (TOP_CAP_DEPTH - 0.044) * 0.5,
                (TOP_CAP_THICKNESS - TOP_CAP_LIP_DROP) * 0.5,
            )
        ),
        material=drawer_face,
        name="right_return",
    )
    top_cap.inertial = Inertial.from_geometry(
        Box((TOP_CAP_WIDTH, TOP_CAP_DEPTH, TOP_CAP_LIP_DROP)),
        mass=3.8,
        origin=Origin(xyz=(0.0, TOP_CAP_DEPTH * 0.5, 0.004)),
    )

    model.articulation(
        "body_to_top_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=top_cap,
        origin=Origin(xyz=(0.0, TOP_CAP_HINGE_Y, BODY_BASE_Z + BODY_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    top_cap = object_model.get_part("top_cap")
    top_hinge = object_model.get_articulation("body_to_top_cap")

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

    ctx.check(
        "top cap hinge axis pitches from rear edge",
        tuple(round(value, 6) for value in top_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={top_hinge.axis}",
    )
    ctx.expect_contact(
        top_cap,
        body,
        elem_a="cap_slab",
        contact_tol=0.0005,
        name="top cap sits on the cabinet rim",
    )

    closed_front = ctx.part_element_world_aabb(top_cap, elem="front_return")
    with ctx.pose({top_hinge: top_hinge.motion_limits.upper or 0.0}):
        open_front = ctx.part_element_world_aabb(top_cap, elem="front_return")
    ctx.check(
        "top cap opens upward for mechanism access",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.20,
        details=f"closed_front={closed_front}, open_front={open_front}",
    )

    for drawer_number in range(1, DRAWER_COUNT + 1):
        drawer = object_model.get_part(f"drawer_{drawer_number}")
        slide = object_model.get_articulation(f"body_to_drawer_{drawer_number}")
        limits = slide.motion_limits

        ctx.check(
            f"drawer {drawer_number} slides forward on prismatic axis",
            tuple(round(value, 6) for value in slide.axis) == (0.0, 1.0, 0.0),
            details=f"axis={slide.axis}",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="inner_left_slide",
            elem_b=f"outer_left_slide_{drawer_number}",
            contact_tol=0.0005,
            name=f"drawer {drawer_number} left slide is mounted to cabinet rail",
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a="inner_right_slide",
            elem_b=f"outer_right_slide_{drawer_number}",
            contact_tol=0.0005,
            name=f"drawer {drawer_number} right slide is mounted to cabinet rail",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="inner_left_slide",
            elem_b=f"outer_left_slide_{drawer_number}",
            min_overlap=0.34,
            name=f"drawer {drawer_number} left slide has deep insertion when closed",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="z",
            elem_a="inner_left_slide",
            elem_b=f"outer_left_slide_{drawer_number}",
            min_overlap=0.038,
            name=f"drawer {drawer_number} left slide vertical engagement is aligned",
        )

        closed_position = ctx.part_world_position(drawer)
        with ctx.pose({slide: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            ctx.expect_contact(
                drawer,
                body,
                elem_a="inner_left_slide",
                elem_b=f"outer_left_slide_{drawer_number}",
                contact_tol=0.0005,
                name=f"drawer {drawer_number} left slide stays guided at full extension",
            )
            ctx.expect_contact(
                drawer,
                body,
                elem_a="inner_right_slide",
                elem_b=f"outer_right_slide_{drawer_number}",
                contact_tol=0.0005,
                name=f"drawer {drawer_number} right slide stays guided at full extension",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="y",
                elem_a="inner_left_slide",
                elem_b=f"outer_left_slide_{drawer_number}",
                min_overlap=0.030,
                name=f"drawer {drawer_number} retains insertion at full extension",
            )
            open_position = ctx.part_world_position(drawer)

        ctx.check(
            f"drawer {drawer_number} opens outward",
            closed_position is not None
            and open_position is not None
            and open_position[1] > closed_position[1] + 0.30,
            details=f"closed_position={closed_position}, open_position={open_position}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
