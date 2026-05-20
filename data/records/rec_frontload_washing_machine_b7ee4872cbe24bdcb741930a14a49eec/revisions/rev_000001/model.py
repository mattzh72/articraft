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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


WASHER_WIDTH = 0.60
WASHER_DEPTH = 0.66
WASHER_HEIGHT = 0.85
SIDE_THICKNESS = 0.022
TOP_THICKNESS = 0.022
BOTTOM_THICKNESS = 0.030
BACK_THICKNESS = 0.018
FRONT_THICKNESS = 0.018

DOOR_CENTER_Z = 0.390
DOOR_OUTER_RADIUS = 0.255
DOOR_CLEAR_OPENING_RADIUS = 0.198
DOOR_GLASS_RADIUS = 0.178
DOOR_HINGE_OFFSET = 0.057
DOOR_HINGE_FRONT_OFFSET = 0.012
DOOR_CENTER_FROM_HINGE = DOOR_OUTER_RADIUS + DOOR_HINGE_OFFSET

DRAWER_CENTER_X = -0.168
DRAWER_CENTER_Z = 0.734
DRAWER_FACE_WIDTH = 0.194
DRAWER_FACE_HEIGHT = 0.100
DRAWER_FACE_THICKNESS = 0.012
DRAWER_BIN_DEPTH = 0.360
DRAWER_BIN_OUTER_WIDTH = 0.192
DRAWER_BIN_HEIGHT = 0.076


def _rectangle_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 72,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    cx, cy = center
    points: list[tuple[float, float]] = []
    for index in range(segments):
        angle = math.tau * index / segments
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    if clockwise:
        points.reverse()
    return points


def _annulus_mesh(outer_radius: float, inner_radius: float, thickness: float, name: str):
    geometry = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=96),
        [_circle_profile(inner_radius, segments=96, clockwise=True)],
        thickness,
        cap=True,
        center=True,
        closed=True,
    )
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="front_load_washing_machine")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.94, 0.95, 1.0))
    enamel_shadow = model.material("enamel_shadow", rgba=(0.82, 0.84, 0.87, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.58, 0.61, 0.66, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    gasket_grey = model.material("gasket_grey", rgba=(0.47, 0.49, 0.52, 1.0))
    display_black = model.material("display_black", rgba=(0.10, 0.11, 0.12, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.62, 0.72, 0.80, 0.28))
    drawer_plastic = model.material("drawer_plastic", rgba=(0.88, 0.89, 0.90, 1.0))
    detergent_handle = model.material("detergent_handle", rgba=(0.76, 0.79, 0.82, 1.0))

    cabinet = model.part("cabinet")

    front_panel = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rectangle_profile(WASHER_WIDTH, WASHER_HEIGHT),
            [
                _circle_profile(
                    DOOR_CLEAR_OPENING_RADIUS,
                    center=(0.0, DOOR_CENTER_Z - WASHER_HEIGHT * 0.5),
                    segments=96,
                    clockwise=True,
                ),
                list(
                    reversed(
                        [
                            (
                                x + DRAWER_CENTER_X,
                                y + (DRAWER_CENTER_Z - WASHER_HEIGHT * 0.5),
                            )
                            for x, y in rounded_rect_profile(
                                0.182,
                                0.086,
                                0.010,
                                corner_segments=5,
                            )
                        ]
                    )
                ),
            ],
            FRONT_THICKNESS,
            cap=True,
            center=True,
            closed=True,
        ),
        "washer_front_fascia",
    )
    cabinet.visual(
        front_panel,
        origin=Origin(
            xyz=(0.0, WASHER_DEPTH * 0.5 - FRONT_THICKNESS * 0.5, WASHER_HEIGHT * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=cabinet_white,
        name="front_fascia",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, WASHER_DEPTH, WASHER_HEIGHT)),
        origin=Origin(
            xyz=(-WASHER_WIDTH * 0.5 + SIDE_THICKNESS * 0.5, 0.0, WASHER_HEIGHT * 0.5)
        ),
        material=cabinet_white,
        name="left_side_panel",
    )
    cabinet.visual(
        Box((SIDE_THICKNESS, WASHER_DEPTH, WASHER_HEIGHT)),
        origin=Origin(
            xyz=(WASHER_WIDTH * 0.5 - SIDE_THICKNESS * 0.5, 0.0, WASHER_HEIGHT * 0.5)
        ),
        material=cabinet_white,
        name="right_side_panel",
    )
    cabinet.visual(
        Box((WASHER_WIDTH - 2.0 * SIDE_THICKNESS, WASHER_DEPTH, TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, WASHER_HEIGHT - TOP_THICKNESS * 0.5)),
        material=cabinet_white,
        name="top_panel",
    )
    cabinet.visual(
        Box((WASHER_WIDTH - 2.0 * SIDE_THICKNESS, WASHER_DEPTH, BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_THICKNESS * 0.5)),
        material=enamel_shadow,
        name="bottom_pan",
    )
    cabinet.visual(
        Box(
            (
                WASHER_WIDTH - 2.0 * SIDE_THICKNESS,
                BACK_THICKNESS,
                WASHER_HEIGHT - TOP_THICKNESS - BOTTOM_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                -WASHER_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                0.5 * (BOTTOM_THICKNESS + (WASHER_HEIGHT - TOP_THICKNESS)),
            )
        ),
        material=enamel_shadow,
        name="rear_panel",
    )
    cabinet.visual(
        _annulus_mesh(0.208, 0.178, 0.020, "door_boot_gasket"),
        origin=Origin(
            xyz=(0.0, WASHER_DEPTH * 0.5 - FRONT_THICKNESS - 0.010, DOOR_CENTER_Z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=gasket_grey,
        name="door_boot",
    )
    cabinet.visual(
        Box((0.242, 0.402, 0.004)),
        origin=Origin(xyz=(DRAWER_CENTER_X, 0.111, 0.687)),
        material=enamel_shadow,
        name="drawer_bay_floor",
    )
    cabinet.visual(
        Box((0.242, 0.402, 0.004)),
        origin=Origin(xyz=(DRAWER_CENTER_X, 0.111, 0.781)),
        material=enamel_shadow,
        name="drawer_bay_roof",
    )
    cabinet.visual(
        Box((0.006, 0.402, 0.094)),
        origin=Origin(xyz=(-0.047, 0.111, 0.734)),
        material=enamel_shadow,
        name="drawer_bay_divider",
    )
    cabinet.visual(
        Box((0.010, 0.360, 0.012)),
        origin=Origin(xyz=(-0.274, 0.133, 0.734)),
        material=trim_grey,
        name="drawer_left_rail",
    )
    cabinet.visual(
        Box((0.010, 0.360, 0.012)),
        origin=Origin(xyz=(-0.054, 0.133, 0.734)),
        material=trim_grey,
        name="drawer_right_rail",
    )
    cabinet.visual(
        Box((0.150, 0.008, 0.054)),
        origin=Origin(xyz=(0.075, WASHER_DEPTH * 0.5 + 0.004, 0.742)),
        material=display_black,
        name="display_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.048, length=0.028),
        origin=Origin(
            xyz=(0.202, WASHER_DEPTH * 0.5 + 0.014, 0.736),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="program_knob",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.140),
        origin=Origin(
            xyz=(
                -DOOR_CENTER_FROM_HINGE,
                WASHER_DEPTH * 0.5 + DOOR_HINGE_FRONT_OFFSET,
                DOOR_CENTER_Z - 0.165,
            )
        ),
        material=trim_grey,
        name="cabinet_hinge_barrel_lower",
    )
    cabinet.visual(
        Box((0.024, 0.024, 0.082)),
        origin=Origin(
            xyz=(
                -DOOR_CENTER_FROM_HINGE + 0.012,
                WASHER_DEPTH * 0.5,
                DOOR_CENTER_Z - 0.165,
            )
        ),
        material=trim_grey,
        name="cabinet_hinge_mount_lower",
    )
    cabinet.visual(
        Cylinder(radius=0.012, length=0.140),
        origin=Origin(
            xyz=(
                -DOOR_CENTER_FROM_HINGE,
                WASHER_DEPTH * 0.5 + DOOR_HINGE_FRONT_OFFSET,
                DOOR_CENTER_Z + 0.070,
            )
        ),
        material=trim_grey,
        name="cabinet_hinge_barrel_upper",
    )
    cabinet.visual(
        Box((0.024, 0.024, 0.082)),
        origin=Origin(
            xyz=(
                -DOOR_CENTER_FROM_HINGE + 0.012,
                WASHER_DEPTH * 0.5,
                DOOR_CENTER_Z + 0.070,
            )
        ),
        material=trim_grey,
        name="cabinet_hinge_mount_upper",
    )
    cabinet.inertial = Inertial.from_geometry(
        Box((WASHER_WIDTH, WASHER_DEPTH, WASHER_HEIGHT)),
        mass=54.0,
        origin=Origin(xyz=(0.0, 0.0, WASHER_HEIGHT * 0.5)),
    )

    door = model.part("door")
    door.visual(
        _annulus_mesh(0.255, 0.195, 0.024, "door_outer_ring"),
        origin=Origin(
            xyz=(DOOR_CENTER_FROM_HINGE, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_grey,
        name="door_outer_ring",
    )
    door.visual(
        _annulus_mesh(0.222, 0.165, 0.014, "door_inner_ring"),
        origin=Origin(
            xyz=(DOOR_CENTER_FROM_HINGE, 0.018, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_trim,
        name="door_inner_ring",
    )
    glass_profile = [
        (0.0, 0.000),
        (0.150, 0.000),
        (0.170, -0.010),
        (DOOR_GLASS_RADIUS, -0.030),
        (0.170, -0.046),
        (0.096, -0.052),
        (0.0, -0.050),
    ]
    door.visual(
        mesh_from_geometry(LatheGeometry(glass_profile, segments=88, closed=True), "door_glass_lens"),
        origin=Origin(
            xyz=(DOOR_CENTER_FROM_HINGE, 0.012, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=glass_clear,
        name="door_glass",
    )
    door.visual(
        Box((0.132, 0.028, 0.060)),
        origin=Origin(xyz=(0.074, 0.008, -0.047)),
        material=dark_trim,
        name="door_hinge_arm_lower",
    )
    door.visual(
        Box((0.118, 0.028, 0.050)),
        origin=Origin(xyz=(0.067, 0.008, 0.173)),
        material=dark_trim,
        name="door_hinge_arm_upper",
    )
    door.visual(
        Box((0.072, 0.024, 0.246)),
        origin=Origin(xyz=(0.118, 0.006, 0.055)),
        material=dark_trim,
        name="door_hinge_web",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, -0.0475)),
        material=trim_grey,
        name="door_hinge_barrel_lower",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        material=trim_grey,
        name="door_hinge_barrel_upper",
    )
    door.visual(
        Box((0.026, 0.030, 0.130)),
        origin=Origin(xyz=(0.468 + DOOR_HINGE_OFFSET, 0.027, 0.0)),
        material=dark_trim,
        name="door_handle",
    )
    door.inertial = Inertial.from_geometry(
        Box((0.514, 0.070, 0.514)),
        mass=3.8,
        origin=Origin(xyz=(0.257, 0.018, 0.0)),
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((DRAWER_FACE_WIDTH, DRAWER_FACE_THICKNESS, DRAWER_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_THICKNESS * 0.5, 0.0)),
        material=drawer_plastic,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.110, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, DRAWER_FACE_THICKNESS + 0.005, 0.0)),
        material=detergent_handle,
        name="drawer_pull",
    )
    drawer.visual(
        Box((0.184, 0.354, 0.004)),
        origin=Origin(xyz=(0.0, -0.179, -0.036)),
        material=drawer_plastic,
        name="drawer_bin_floor",
    )
    drawer.visual(
        Box((0.004, DRAWER_BIN_DEPTH, 0.072)),
        origin=Origin(xyz=(-0.094, -0.180, -0.002)),
        material=drawer_plastic,
        name="drawer_left_wall",
    )
    drawer.visual(
        Box((0.004, DRAWER_BIN_DEPTH, 0.072)),
        origin=Origin(xyz=(0.094, -0.180, -0.002)),
        material=drawer_plastic,
        name="drawer_right_wall",
    )
    drawer.visual(
        Box((0.184, 0.004, 0.072)),
        origin=Origin(xyz=(0.0, -0.002, -0.002)),
        material=drawer_plastic,
        name="drawer_front_wall",
    )
    drawer.visual(
        Box((0.184, 0.004, 0.072)),
        origin=Origin(xyz=(0.0, -0.358, -0.002)),
        material=drawer_plastic,
        name="drawer_back_wall",
    )
    drawer.visual(
        Box((0.004, 0.246, 0.058)),
        origin=Origin(xyz=(-0.030, -0.187, -0.009)),
        material=drawer_plastic,
        name="drawer_divider_left",
    )
    drawer.visual(
        Box((0.004, 0.246, 0.058)),
        origin=Origin(xyz=(0.030, -0.187, -0.009)),
        material=drawer_plastic,
        name="drawer_divider_right",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_BIN_OUTER_WIDTH, DRAWER_BIN_DEPTH + DRAWER_FACE_THICKNESS, DRAWER_BIN_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(0.0, -0.172, -0.004)),
    )

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(
            xyz=(
                -DOOR_CENTER_FROM_HINGE,
                WASHER_DEPTH * 0.5 + DOOR_HINGE_FRONT_OFFSET,
                DOOR_CENTER_Z,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "cabinet_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(
            xyz=(
                DRAWER_CENTER_X,
                WASHER_DEPTH * 0.5 + 0.001,
                DRAWER_CENTER_Z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.28,
            lower=0.0,
            upper=0.240,
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

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drawer = object_model.get_part("detergent_drawer")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drawer_slide = object_model.get_articulation("cabinet_to_detergent_drawer")

    ctx.check(
        "door hinge uses a vertical axis",
        tuple(round(value, 3) for value in door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "drawer slides outward along the cabinet depth axis",
        tuple(round(value, 3) for value in drawer_slide.axis) == (0.0, 1.0, 0.0),
        details=f"axis={drawer_slide.axis}",
    )

    with ctx.pose({door_hinge: 0.0, drawer_slide: 0.0}):
        ctx.expect_gap(
            door,
            cabinet,
            axis="y",
            positive_elem="door_outer_ring",
            negative_elem="front_fascia",
            min_gap=0.0,
            max_gap=0.008,
            name="closed door sits just proud of the front fascia",
        )
        ctx.expect_overlap(
            door,
            cabinet,
            axes="xz",
            elem_a="door_outer_ring",
            elem_b="front_fascia",
            min_overlap=0.44,
            name="closed door covers the porthole opening",
        )

        closed_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")
        with ctx.pose({door_hinge: 1.85}):
            open_handle_aabb = ctx.part_element_world_aabb(door, elem="door_handle")

        closed_handle_max_y = None if closed_handle_aabb is None else closed_handle_aabb[1][1]
        open_handle_max_y = None if open_handle_aabb is None else open_handle_aabb[1][1]
        ctx.check(
            "door swings forward when opened",
            closed_handle_max_y is not None
            and open_handle_max_y is not None
            and open_handle_max_y > closed_handle_max_y + 0.14,
            details=f"closed_handle_max_y={closed_handle_max_y}, open_handle_max_y={open_handle_max_y}",
        )

        rest_drawer_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_slide: 0.240}):
            extended_drawer_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                cabinet,
                axes="y",
                elem_a="drawer_bin_floor",
                elem_b="drawer_left_rail",
                min_overlap=0.09,
                name="drawer remains engaged on the guide rails at full extension",
            )

        ctx.check(
            "drawer extends forward from the cabinet",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.18,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
