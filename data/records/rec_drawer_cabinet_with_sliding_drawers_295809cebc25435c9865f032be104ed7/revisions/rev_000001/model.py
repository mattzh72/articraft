from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BODY_WIDTH = 0.82
BODY_DEPTH = 0.46
SHELL_HEIGHT = 0.83
SHELL_BOTTOM_Z = 0.098
SHELL_TOP_Z = SHELL_BOTTOM_Z + SHELL_HEIGHT
BODY_HALF_WIDTH = BODY_WIDTH * 0.5
BODY_HALF_DEPTH = BODY_DEPTH * 0.5
PANEL_THICKNESS = 0.003
TOP_PANEL_THICKNESS = 0.004
BOTTOM_PANEL_THICKNESS = 0.004
FRONT_FRAME_THICKNESS = 0.010
FRONT_STILE_WIDTH = 0.015
OPENING_BOTTOM_Z = 0.122
OPENING_TOP_Z = 0.904
DRAWER_FRONT_WIDTH = 0.788
DRAWER_OUTER_WIDTH = 0.760
DRAWER_DEPTH = 0.438
DRAWER_HALF_DEPTH = DRAWER_DEPTH * 0.5
DRAWER_FRONT_THICKNESS = 0.018
DRAWER_BACK_THICKNESS = 0.008
DRAWER_SIDE_THICKNESS = 0.010
DRAWER_BASE_THICKNESS = 0.004
DRAWER_TRAVEL = 0.36
DRAWER_CLOSED_CENTER_X = BODY_HALF_DEPTH - DRAWER_HALF_DEPTH
SLIDE_GAP = 0.027
BODY_SLIDE_THICKNESS = 0.015
DRAWER_SLIDE_THICKNESS = 0.012
SLIDE_HEIGHT = 0.018
SLIDE_LENGTH = 0.39
BODY_SLIDE_CENTER_X = -0.013
DRAWER_SLIDE_CENTER_X = BODY_SLIDE_CENTER_X - DRAWER_CLOSED_CENTER_X
DRAWER_FRONT_HEIGHTS = (0.070, 0.070, 0.086, 0.100, 0.114, 0.128, 0.174)
TOP_MARGIN = 0.008
BOTTOM_MARGIN = 0.008
DRAWER_GAP = 0.004


def _drawer_mass(front_height: float) -> float:
    return 2.8 + front_height * 17.0


def _drawer_slide_z(front_height: float) -> float:
    return -front_height * 0.5 + 0.028


def _add_caster(
    body,
    prefix: str,
    *,
    x: float,
    y: float,
    shell_bottom_z: float,
    wheel_material,
    fork_material,
    plate_material,
) -> None:
    plate_t = 0.006
    wheel_radius = 0.040
    wheel_width = 0.028
    fork_plate_t = 0.004
    fork_plate_y = wheel_width * 0.5 + fork_plate_t * 0.5

    body.visual(
        Box((0.058, 0.044, plate_t)),
        origin=Origin(xyz=(x, y, shell_bottom_z - plate_t * 0.5)),
        material=plate_material,
        name=f"{prefix}_mount_plate",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(x, y, 0.087)),
        material=fork_material,
        name=f"{prefix}_swivel_stem",
    )
    body.visual(
        Box((0.038, 0.036, 0.008)),
        origin=Origin(xyz=(x, y, 0.078)),
        material=fork_material,
        name=f"{prefix}_yoke",
    )
    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        body.visual(
            Box((0.050, fork_plate_t, 0.040)),
            origin=Origin(xyz=(x, y + side_sign * fork_plate_y, 0.060)),
            material=fork_material,
            name=f"{prefix}_{side_name}_fork",
        )
    body.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(x, y, wheel_radius), rpy=(pi * 0.5, 0.0, 0.0)),
        material=plate_material,
        name=f"{prefix}_axle",
    )
    body.visual(
        Cylinder(radius=wheel_radius, length=wheel_width),
        origin=Origin(xyz=(x, y, wheel_radius), rpy=(pi * 0.5, 0.0, 0.0)),
        material=wheel_material,
        name=f"{prefix}_wheel",
    )
    lever_x = x + 0.015 if x > 0.0 else x - 0.015
    lever_y = y + 0.010 if y > 0.0 else y - 0.010
    body.visual(
        Box((0.018, 0.010, 0.004)),
        origin=Origin(xyz=(lever_x, lever_y, 0.082)),
        material=plate_material,
        name=f"{prefix}_lock_lever",
    )


def _populate_drawer(drawer, *, front_height: float, front_material, tray_material, slide_material, handle_material) -> None:
    half_height = front_height * 0.5
    tray_depth = DRAWER_DEPTH - DRAWER_FRONT_THICKNESS - DRAWER_BACK_THICKNESS
    tray_center_x = -0.005
    side_height = front_height - 0.018
    side_center_z = -half_height + DRAWER_BASE_THICKNESS + side_height * 0.5
    front_center_x = DRAWER_HALF_DEPTH - DRAWER_FRONT_THICKNESS * 0.5
    back_center_x = -DRAWER_HALF_DEPTH + DRAWER_BACK_THICKNESS * 0.5

    drawer.visual(
        Box((tray_depth, DRAWER_OUTER_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, DRAWER_BASE_THICKNESS)),
        origin=Origin(xyz=(tray_center_x, 0.0, -half_height + DRAWER_BASE_THICKNESS * 0.5)),
        material=tray_material,
        name="drawer_base",
    )
    drawer.visual(
        Box((tray_depth, DRAWER_SIDE_THICKNESS, side_height)),
        origin=Origin(
            xyz=(
                tray_center_x,
                -DRAWER_OUTER_WIDTH * 0.5 + DRAWER_SIDE_THICKNESS * 0.5,
                side_center_z,
            )
        ),
        material=tray_material,
        name="left_sidewall",
    )
    drawer.visual(
        Box((tray_depth, DRAWER_SIDE_THICKNESS, side_height)),
        origin=Origin(
            xyz=(
                tray_center_x,
                DRAWER_OUTER_WIDTH * 0.5 - DRAWER_SIDE_THICKNESS * 0.5,
                side_center_z,
            )
        ),
        material=tray_material,
        name="right_sidewall",
    )
    drawer.visual(
        Box((DRAWER_BACK_THICKNESS, DRAWER_OUTER_WIDTH - 2.0 * DRAWER_SIDE_THICKNESS, side_height)),
        origin=Origin(xyz=(back_center_x, 0.0, side_center_z)),
        material=tray_material,
        name="backwall",
    )

    pocket_width = DRAWER_FRONT_WIDTH * 0.78
    pocket_height = max(0.020, min(front_height * 0.32, front_height - 0.036))
    top_strip_height = max(0.018, min(front_height * 0.24, front_height - pocket_height - 0.024))
    bottom_strip_height = front_height - pocket_height - top_strip_height
    side_strip_width = (DRAWER_FRONT_WIDTH - pocket_width) * 0.5
    pocket_floor_thickness = DRAWER_FRONT_THICKNESS - 0.008
    pocket_floor_center_x = front_center_x - DRAWER_FRONT_THICKNESS * 0.5 + pocket_floor_thickness * 0.5
    top_strip_center_z = half_height - top_strip_height * 0.5
    bottom_strip_center_z = -half_height + bottom_strip_height * 0.5
    pocket_center_z = -half_height + bottom_strip_height + pocket_height * 0.5

    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, top_strip_height)),
        origin=Origin(xyz=(front_center_x, 0.0, top_strip_center_z)),
        material=front_material,
        name="front_top_strip",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, DRAWER_FRONT_WIDTH, bottom_strip_height)),
        origin=Origin(xyz=(front_center_x, 0.0, bottom_strip_center_z)),
        material=front_material,
        name="front_bottom_strip",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, side_strip_width, pocket_height)),
        origin=Origin(
            xyz=(
                front_center_x,
                -DRAWER_FRONT_WIDTH * 0.5 + side_strip_width * 0.5,
                pocket_center_z,
            )
        ),
        material=front_material,
        name="front_left_strip",
    )
    drawer.visual(
        Box((DRAWER_FRONT_THICKNESS, side_strip_width, pocket_height)),
        origin=Origin(
            xyz=(
                front_center_x,
                DRAWER_FRONT_WIDTH * 0.5 - side_strip_width * 0.5,
                pocket_center_z,
            )
        ),
        material=front_material,
        name="front_right_strip",
    )
    drawer.visual(
        Box((pocket_floor_thickness, pocket_width, pocket_height)),
        origin=Origin(xyz=(pocket_floor_center_x, 0.0, pocket_center_z)),
        material=front_material,
        name="handle_recess_floor",
    )
    handle_radius = min(0.006, pocket_height * 0.23)
    drawer.visual(
        Cylinder(radius=handle_radius, length=pocket_width * 0.90),
        origin=Origin(
            xyz=(
                front_center_x + DRAWER_FRONT_THICKNESS * 0.5 - handle_radius * 1.1,
                0.0,
                pocket_center_z + pocket_height * 0.12,
            ),
            rpy=(pi * 0.5, 0.0, 0.0),
        ),
        material=handle_material,
        name="handle_bar",
    )

    left_slide_center_y = -DRAWER_OUTER_WIDTH * 0.5 - DRAWER_SLIDE_THICKNESS * 0.5
    right_slide_center_y = DRAWER_OUTER_WIDTH * 0.5 + DRAWER_SLIDE_THICKNESS * 0.5
    slide_center_z = _drawer_slide_z(front_height)
    for y_center, slide_name in ((left_slide_center_y, "left_slide"), (right_slide_center_y, "right_slide")):
        drawer.visual(
            Box((SLIDE_LENGTH, DRAWER_SLIDE_THICKNESS, SLIDE_HEIGHT)),
            origin=Origin(xyz=(DRAWER_SLIDE_CENTER_X, y_center, slide_center_z)),
            material=slide_material,
            name=slide_name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_tool_cabinet")

    cabinet_red = model.material("cabinet_red", rgba=(0.70, 0.11, 0.10, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.23, 0.24, 0.27, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    zinc = model.material("zinc", rgba=(0.78, 0.79, 0.81, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.84, 0.84, 0.85, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    top_mat = model.material("top_mat", rgba=(0.12, 0.12, 0.13, 1.0))

    drawer_specs = []
    running_top = OPENING_TOP_Z - TOP_MARGIN
    for index, front_height in enumerate(DRAWER_FRONT_HEIGHTS, start=1):
        center_z = running_top - front_height * 0.5
        drawer_name = f"drawer_{index}"
        drawer_specs.append(
            {
                "name": drawer_name,
                "front_height": front_height,
                "center_z": center_z,
            }
        )
        running_top -= front_height + DRAWER_GAP

    body = model.part("cabinet_body")
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, SHELL_TOP_Z)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, SHELL_TOP_Z * 0.5)),
    )

    body.visual(
        Box((BODY_DEPTH, PANEL_THICKNESS, SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, -BODY_HALF_WIDTH + PANEL_THICKNESS * 0.5, SHELL_BOTTOM_Z + SHELL_HEIGHT * 0.5)),
        material=cabinet_red,
        name="left_side_panel",
    )
    body.visual(
        Box((BODY_DEPTH, PANEL_THICKNESS, SHELL_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_HALF_WIDTH - PANEL_THICKNESS * 0.5, SHELL_BOTTOM_Z + SHELL_HEIGHT * 0.5)),
        material=cabinet_red,
        name="right_side_panel",
    )
    body.visual(
        Box((PANEL_THICKNESS, BODY_WIDTH - 2.0 * PANEL_THICKNESS, SHELL_HEIGHT)),
        origin=Origin(xyz=(-BODY_HALF_DEPTH + PANEL_THICKNESS * 0.5, 0.0, SHELL_BOTTOM_Z + SHELL_HEIGHT * 0.5)),
        material=cabinet_red,
        name="back_panel",
    )
    body.visual(
        Box((BODY_DEPTH - PANEL_THICKNESS, BODY_WIDTH - 2.0 * PANEL_THICKNESS, BOTTOM_PANEL_THICKNESS)),
        origin=Origin(xyz=(PANEL_THICKNESS * 0.5, 0.0, SHELL_BOTTOM_Z + BOTTOM_PANEL_THICKNESS * 0.5)),
        material=charcoal,
        name="bottom_pan",
    )
    body.visual(
        Box((BODY_DEPTH - PANEL_THICKNESS, BODY_WIDTH - 2.0 * PANEL_THICKNESS, TOP_PANEL_THICKNESS)),
        origin=Origin(xyz=(PANEL_THICKNESS * 0.5, 0.0, SHELL_TOP_Z - TOP_PANEL_THICKNESS * 0.5)),
        material=cabinet_red,
        name="top_panel",
    )
    body.visual(
        Box((BODY_DEPTH - 0.020, BODY_WIDTH - 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, SHELL_TOP_Z + 0.003)),
        material=top_mat,
        name="worktop_pad",
    )

    frame_height = OPENING_TOP_Z - OPENING_BOTTOM_Z
    body.visual(
        Box((FRONT_FRAME_THICKNESS, FRONT_STILE_WIDTH, frame_height)),
        origin=Origin(
            xyz=(
                BODY_HALF_DEPTH - FRONT_FRAME_THICKNESS * 0.5,
                -BODY_HALF_WIDTH + FRONT_STILE_WIDTH * 0.5,
                OPENING_BOTTOM_Z + frame_height * 0.5,
            )
        ),
        material=cabinet_red,
        name="left_front_stile",
    )
    body.visual(
        Box((FRONT_FRAME_THICKNESS, FRONT_STILE_WIDTH, frame_height)),
        origin=Origin(
            xyz=(
                BODY_HALF_DEPTH - FRONT_FRAME_THICKNESS * 0.5,
                BODY_HALF_WIDTH - FRONT_STILE_WIDTH * 0.5,
                OPENING_BOTTOM_Z + frame_height * 0.5,
            )
        ),
        material=cabinet_red,
        name="right_front_stile",
    )
    body.visual(
        Box((FRONT_FRAME_THICKNESS, BODY_WIDTH - 2.0 * FRONT_STILE_WIDTH, OPENING_BOTTOM_Z - SHELL_BOTTOM_Z)),
        origin=Origin(
            xyz=(
                BODY_HALF_DEPTH - FRONT_FRAME_THICKNESS * 0.5,
                0.0,
                SHELL_BOTTOM_Z + (OPENING_BOTTOM_Z - SHELL_BOTTOM_Z) * 0.5,
            )
        ),
        material=cabinet_red,
        name="bottom_front_rail",
    )
    body.visual(
        Box((FRONT_FRAME_THICKNESS, BODY_WIDTH - 2.0 * FRONT_STILE_WIDTH, SHELL_TOP_Z - OPENING_TOP_Z)),
        origin=Origin(
            xyz=(
                BODY_HALF_DEPTH - FRONT_FRAME_THICKNESS * 0.5,
                0.0,
                OPENING_TOP_Z + (SHELL_TOP_Z - OPENING_TOP_Z) * 0.5,
            )
        ),
        material=cabinet_red,
        name="top_front_rail",
    )

    for spec in drawer_specs:
        slide_center_z = spec["center_z"] + _drawer_slide_z(spec["front_height"])
        left_body_slide_y = -DRAWER_OUTER_WIDTH * 0.5 - DRAWER_SLIDE_THICKNESS - BODY_SLIDE_THICKNESS * 0.5
        right_body_slide_y = DRAWER_OUTER_WIDTH * 0.5 + DRAWER_SLIDE_THICKNESS + BODY_SLIDE_THICKNESS * 0.5
        body.visual(
            Box((SLIDE_LENGTH, BODY_SLIDE_THICKNESS, SLIDE_HEIGHT)),
            origin=Origin(xyz=(BODY_SLIDE_CENTER_X, left_body_slide_y, slide_center_z)),
            material=zinc,
            name=f"{spec['name']}_left_outer_slide",
        )
        body.visual(
            Box((SLIDE_LENGTH, BODY_SLIDE_THICKNESS, SLIDE_HEIGHT)),
            origin=Origin(xyz=(BODY_SLIDE_CENTER_X, right_body_slide_y, slide_center_z)),
            material=zinc,
            name=f"{spec['name']}_right_outer_slide",
        )

    caster_x = BODY_HALF_DEPTH - 0.075
    caster_y = BODY_HALF_WIDTH - 0.075
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            x = caster_x * x_sign
            y = caster_y * y_sign
            name = f"{'front' if x_sign > 0.0 else 'rear'}_{'right' if y_sign > 0.0 else 'left'}_caster"
            _add_caster(
                body,
                name,
                x=x,
                y=y,
                shell_bottom_z=SHELL_BOTTOM_Z,
                wheel_material=rubber,
                fork_material=charcoal,
                plate_material=zinc,
            )

    for spec in drawer_specs:
        drawer = model.part(
            spec["name"],
            meta={
                "front_height": spec["front_height"],
                "slide_extension": DRAWER_TRAVEL,
            },
        )
        drawer.inertial = Inertial.from_geometry(
            Box((DRAWER_DEPTH, DRAWER_FRONT_WIDTH, spec["front_height"])),
            mass=_drawer_mass(spec["front_height"]),
        )
        _populate_drawer(
            drawer,
            front_height=spec["front_height"],
            front_material=cabinet_red,
            tray_material=tray_gray,
            slide_material=zinc,
            handle_material=brushed_aluminum,
        )
        model.articulation(
            f"{spec['name']}_slide",
            ArticulationType.PRISMATIC,
            parent=body,
            child=drawer,
            origin=Origin(xyz=(DRAWER_CLOSED_CENTER_X, 0.0, spec["center_z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=240.0,
                velocity=0.65,
                lower=0.0,
                upper=DRAWER_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    drawer_names = [f"drawer_{index}" for index in range(1, 8)]
    drawers = [object_model.get_part(name) for name in drawer_names]
    slides = [object_model.get_articulation(f"{name}_slide") for name in drawer_names]

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

    front_heights = [drawer.meta.get("front_height") for drawer in drawers]
    ctx.check(
        "drawer_fronts_graduate_deeper_downward",
        all(
            isinstance(front_heights[index], (int, float))
            and isinstance(front_heights[index + 1], (int, float))
            and front_heights[index] <= front_heights[index + 1]
            for index in range(len(front_heights) - 1)
        ),
        details=f"front heights were {front_heights}",
    )

    for drawer, slide in zip(drawers, slides):
        ctx.check(
            f"{drawer.name}_uses_prismatic_slide",
            slide.articulation_type == ArticulationType.PRISMATIC
            and slide.axis == (1.0, 0.0, 0.0)
            and slide.motion_limits is not None
            and slide.motion_limits.lower == 0.0
            and slide.motion_limits.upper is not None
            and abs(slide.motion_limits.upper - DRAWER_TRAVEL) < 1e-9,
            details=(
                f"type={slide.articulation_type}, axis={slide.axis}, "
                f"limits={slide.motion_limits}"
            ),
        )
        ctx.expect_contact(
            drawer,
            body,
            elem_a=drawer.get_visual("left_slide"),
            elem_b=body.get_visual(f"{drawer.name}_left_outer_slide"),
            contact_tol=0.0005,
            name=f"{drawer.name}_left_slide_supported_when_closed",
        )
        ctx.expect_within(
            drawer,
            body,
            axes="yz",
            margin=0.001,
            name=f"{drawer.name}_stays_within_body_profile",
        )

    top_drawer = drawers[0]
    bottom_drawer = drawers[-1]
    top_slide = slides[0]
    bottom_slide = slides[-1]

    with ctx.pose({top_slide: DRAWER_TRAVEL}):
        ctx.expect_contact(
            top_drawer,
            body,
            elem_a=top_drawer.get_visual("left_slide"),
            elem_b=body.get_visual(f"{top_drawer.name}_left_outer_slide"),
            contact_tol=0.0005,
            name="top_drawer_slide_stays_engaged_when_open",
        )
        ctx.expect_gap(
            top_drawer,
            body,
            axis="x",
            positive_elem=top_drawer.get_visual("front_top_strip"),
            min_gap=0.30,
            max_gap=0.36,
            name="top_drawer_front_projects_forward_when_open",
        )

    with ctx.pose({bottom_slide: DRAWER_TRAVEL}):
        ctx.expect_contact(
            bottom_drawer,
            body,
            elem_a=bottom_drawer.get_visual("left_slide"),
            elem_b=body.get_visual(f"{bottom_drawer.name}_left_outer_slide"),
            contact_tol=0.0005,
            name="bottom_drawer_slide_stays_engaged_when_open",
        )
        ctx.expect_gap(
            bottom_drawer,
            body,
            axis="x",
            positive_elem=bottom_drawer.get_visual("front_top_strip"),
            min_gap=0.30,
            max_gap=0.36,
            name="bottom_drawer_front_projects_forward_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
