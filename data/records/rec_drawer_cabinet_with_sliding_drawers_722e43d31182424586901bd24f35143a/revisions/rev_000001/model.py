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


FRAME_WIDTH = 0.700
FRAME_DEPTH = 0.440
FRAME_TUBE = 0.020
FRAME_HALF_WIDTH = FRAME_WIDTH * 0.5
FRAME_HALF_DEPTH = FRAME_DEPTH * 0.5
FRAME_FRONT_Y = -FRAME_HALF_DEPTH + FRAME_TUBE * 0.5
FRAME_BACK_Y = FRAME_HALF_DEPTH - FRAME_TUBE * 0.5
FRAME_INNER_FRONT_Y = FRAME_FRONT_Y + FRAME_TUBE * 0.5
FRAME_INNER_BACK_Y = FRAME_BACK_Y - FRAME_TUBE * 0.5

BOTTOM_RAIL_Z = 0.102
LOWER_SHELF_Z = 0.124
BANK_BOTTOM_Z = 0.158
TOP_RAIL_Z = 0.850
TOP_DECK_Z = 0.8615
HANDLE_TOP_Z = 1.020

DRAWER_ROWS = 4
DRAWER_COLS = 4
OPENING_WIDTH = 0.150
OPENING_HEIGHT = 0.145
GRID_PITCH_X = OPENING_WIDTH + FRAME_TUBE
GRID_PITCH_Z = OPENING_HEIGHT + FRAME_TUBE

DRAWER_BODY_WIDTH = 0.129
DRAWER_FRONT_WIDTH = 0.142
DRAWER_DEPTH = 0.383
DRAWER_HEIGHT = 0.138
DRAWER_BODY_HEIGHT = 0.120
DRAWER_TRAVEL = 0.155

GUIDE_WIDTH = OPENING_WIDTH
GUIDE_LENGTH = FRAME_INNER_BACK_Y - FRAME_INNER_FRONT_Y
GUIDE_RAIL_WIDTH = 0.010
GUIDE_RAIL_HEIGHT = 0.004
GUIDE_RAIL_X = 0.060


def _drawer_part_name(row: int, col: int) -> str:
    return f"drawer_r{row}c{col}"


def _guide_part_name(row: int, col: int) -> str:
    return f"guide_r{row}c{col}"


def _slide_joint_name(row: int, col: int) -> str:
    return f"slide_r{row}c{col}"


def _guide_joint_name(row: int, col: int) -> str:
    return f"mount_guide_r{row}c{col}"


def _caster_part_name(index: int) -> str:
    return f"caster_{index}"


def _caster_joint_name(index: int) -> str:
    return f"mount_caster_{index}"


def _column_centers() -> list[float]:
    left_boundary = -FRAME_HALF_WIDTH + FRAME_TUBE
    return [
        left_boundary + OPENING_WIDTH * 0.5 + GRID_PITCH_X * col
        for col in range(DRAWER_COLS)
    ]


def _row_floor_zs() -> list[float]:
    return [BANK_BOTTOM_Z + FRAME_TUBE + GRID_PITCH_Z * row for row in range(DRAWER_ROWS)]


def _row_support_zs() -> list[float]:
    return [BANK_BOTTOM_Z + FRAME_TUBE * 0.5 + GRID_PITCH_Z * row for row in range(DRAWER_ROWS + 1)]


def _build_drawer_part(model: ArticulatedObject, name: str, shell_material, grip_material) -> None:
    drawer = model.part(name)

    drawer.visual(
        Box((DRAWER_FRONT_WIDTH, 0.018, 0.136)),
        origin=Origin(xyz=(0.0, -0.009, 0.070)),
        material=shell_material,
        name="front_panel",
    )
    drawer.visual(
        Box((0.123, 0.379, 0.004)),
        origin=Origin(xyz=(0.0, 0.1895, 0.002)),
        material=shell_material,
        name="bin_floor",
    )
    drawer.visual(
        Box((0.003, 0.379, DRAWER_BODY_HEIGHT)),
        origin=Origin(xyz=(-0.063, 0.1895, 0.062)),
        material=shell_material,
        name="left_wall",
    )
    drawer.visual(
        Box((0.003, 0.379, DRAWER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.063, 0.1895, 0.062)),
        material=shell_material,
        name="right_wall",
    )
    drawer.visual(
        Box((0.123, 0.004, DRAWER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.377, 0.062)),
        material=shell_material,
        name="back_wall",
    )
    drawer.visual(
        Box((0.080, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.014, 0.038)),
        material=grip_material,
        name="pull_grip",
    )
    drawer.visual(
        Box((0.066, 0.004, 0.020)),
        origin=Origin(xyz=(0.0, -0.003, 0.094)),
        material=grip_material,
        name="label_tab",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((DRAWER_FRONT_WIDTH, DRAWER_DEPTH, DRAWER_HEIGHT)),
        mass=0.22,
        origin=Origin(xyz=(0.0, DRAWER_DEPTH * 0.5 - 0.005, DRAWER_HEIGHT * 0.5)),
    )


def _build_guide_part(model: ArticulatedObject, name: str, rail_material) -> None:
    guide = model.part(name)

    guide.visual(
        Box((GUIDE_RAIL_WIDTH, 0.390, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_RAIL_X, 0.200, -0.002)),
        material=rail_material,
        name="left_rail",
    )
    guide.visual(
        Box((GUIDE_RAIL_WIDTH, 0.390, GUIDE_RAIL_HEIGHT)),
        origin=Origin(xyz=(GUIDE_RAIL_X, 0.200, -0.002)),
        material=rail_material,
        name="right_rail",
    )
    guide.visual(
        Box((0.024, 0.010, 0.012)),
        origin=Origin(xyz=(-0.063, 0.005, 0.002)),
        material=rail_material,
        name="left_front_tab",
    )
    guide.visual(
        Box((0.024, 0.010, 0.012)),
        origin=Origin(xyz=(0.063, 0.005, 0.002)),
        material=rail_material,
        name="right_front_tab",
    )
    guide.visual(
        Box((GUIDE_WIDTH, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.395, 0.003)),
        material=rail_material,
        name="rear_bridge",
    )
    guide.inertial = Inertial.from_geometry(
        Box((GUIDE_WIDTH, GUIDE_LENGTH, 0.016)),
        mass=0.05,
        origin=Origin(xyz=(0.0, GUIDE_LENGTH * 0.5, 0.003)),
    )


def _build_caster_part(model: ArticulatedObject, name: str, steel_material, wheel_material) -> None:
    caster = model.part(name)

    caster.visual(
        Box((0.055, 0.045, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=steel_material,
        name="top_plate",
    )
    caster.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=steel_material,
        name="swivel_housing",
    )
    caster.visual(
        Box((0.032, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.022)),
        material=steel_material,
        name="fork_yoke",
    )
    caster.visual(
        Box((0.004, 0.028, 0.038)),
        origin=Origin(xyz=(-0.014, 0.0, -0.039)),
        material=steel_material,
        name="left_fork",
    )
    caster.visual(
        Box((0.004, 0.028, 0.038)),
        origin=Origin(xyz=(0.014, 0.0, -0.039)),
        material=steel_material,
        name="right_fork",
    )
    caster.visual(
        Cylinder(radius=0.003, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.056), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel_material,
        name="axle",
    )
    caster.visual(
        Cylinder(radius=0.036, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.056), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=wheel_material,
        name="wheel",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.055, 0.045, 0.096)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_parts_bin_cart")

    powder_coat = model.material("powder_coat", rgba=(0.19, 0.22, 0.24, 1.0))
    shelf_steel = model.material("shelf_steel", rgba=(0.27, 0.30, 0.33, 1.0))
    plated_steel = model.material("plated_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    bin_plastic = model.material("bin_plastic", rgba=(0.89, 0.76, 0.22, 0.90))
    dark_rubber = model.material("dark_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    pull_trim = model.material("pull_trim", rgba=(0.14, 0.15, 0.16, 1.0))

    frame = model.part("cart_frame")

    post_bottom = BOTTOM_RAIL_Z - FRAME_TUBE * 0.5
    post_top = 0.860
    post_length = post_top - post_bottom
    post_center_z = post_bottom + post_length * 0.5

    for x in (-0.340, 0.340):
        for y in (-0.210, 0.210):
            frame.visual(
                Box((FRAME_TUBE, FRAME_TUBE, post_length)),
                origin=Origin(xyz=(x, y, post_center_z)),
                material=powder_coat,
                name=f"corner_post_{'r' if x > 0 else 'l'}_{'b' if y > 0 else 'f'}",
            )

    for y, suffix in ((-0.210, "front"), (0.210, "rear")):
        frame.visual(
            Box((0.680, FRAME_TUBE, FRAME_TUBE)),
            origin=Origin(xyz=(0.0, y, BOTTOM_RAIL_Z)),
            material=powder_coat,
            name=f"bottom_{suffix}_rail",
        )
        frame.visual(
            Box((0.680, FRAME_TUBE, FRAME_TUBE)),
            origin=Origin(xyz=(0.0, y, TOP_RAIL_Z)),
            material=powder_coat,
            name=f"top_{suffix}_rail",
        )

    for x, suffix in ((-0.340, "left"), (0.340, "right")):
        frame.visual(
            Box((FRAME_TUBE, 0.400, FRAME_TUBE)),
            origin=Origin(xyz=(x, 0.0, BOTTOM_RAIL_Z)),
            material=powder_coat,
            name=f"bottom_{suffix}_rail",
        )
        frame.visual(
            Box((FRAME_TUBE, 0.400, FRAME_TUBE)),
            origin=Origin(xyz=(x, 0.0, TOP_RAIL_Z)),
            material=powder_coat,
            name=f"top_{suffix}_rail",
        )

    support_zs = _row_support_zs()
    for z in support_zs:
        frame.visual(
            Box((0.680, FRAME_TUBE, FRAME_TUBE)),
            origin=Origin(xyz=(0.0, FRAME_FRONT_Y, z)),
            material=powder_coat,
            name=f"front_row_support_{int(round(z * 1000.0))}",
        )
        frame.visual(
            Box((0.680, FRAME_TUBE, FRAME_TUBE)),
            origin=Origin(xyz=(0.0, FRAME_BACK_Y, z)),
            material=powder_coat,
            name=f"rear_row_support_{int(round(z * 1000.0))}",
        )
        frame.visual(
            Box((FRAME_TUBE, 0.400, FRAME_TUBE)),
            origin=Origin(xyz=(-0.340, 0.0, z)),
            material=powder_coat,
            name=f"left_side_support_{int(round(z * 1000.0))}",
        )
        frame.visual(
            Box((FRAME_TUBE, 0.400, FRAME_TUBE)),
            origin=Origin(xyz=(0.340, 0.0, z)),
            material=powder_coat,
            name=f"right_side_support_{int(round(z * 1000.0))}",
        )

    for x in (-0.170, 0.0, 0.170):
        frame.visual(
            Box((FRAME_TUBE, FRAME_TUBE, 0.680)),
            origin=Origin(xyz=(x, FRAME_FRONT_Y, 0.498)),
            material=powder_coat,
            name=f"front_divider_{int(round((x + 0.35) * 1000.0))}",
        )
        frame.visual(
            Box((FRAME_TUBE, FRAME_TUBE, 0.680)),
            origin=Origin(xyz=(x, FRAME_BACK_Y, 0.498)),
            material=powder_coat,
            name=f"rear_divider_{int(round((x + 0.35) * 1000.0))}",
        )

    frame.visual(
        Box((0.660, 0.400, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.1135)),
        material=shelf_steel,
        name="lower_shelf",
    )
    frame.visual(
        Box((0.660, 0.400, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, TOP_DECK_Z)),
        material=shelf_steel,
        name="top_tray_floor",
    )
    frame.visual(
        Box((0.620, FRAME_TUBE, 0.018)),
        origin=Origin(xyz=(0.0, -0.190, 0.872)),
        material=shelf_steel,
        name="top_tray_front_lip",
    )
    frame.visual(
        Box((FRAME_TUBE, 0.360, 0.018)),
        origin=Origin(xyz=(-0.320, 0.0, 0.872)),
        material=shelf_steel,
        name="top_tray_left_lip",
    )
    frame.visual(
        Box((FRAME_TUBE, 0.360, 0.018)),
        origin=Origin(xyz=(0.320, 0.0, 0.872)),
        material=shelf_steel,
        name="top_tray_right_lip",
    )

    frame.visual(
        Box((FRAME_TUBE, FRAME_TUBE, 0.160)),
        origin=Origin(xyz=(-0.250, FRAME_BACK_Y, 0.940)),
        material=powder_coat,
        name="left_handle_post",
    )
    frame.visual(
        Box((FRAME_TUBE, FRAME_TUBE, 0.160)),
        origin=Origin(xyz=(0.250, FRAME_BACK_Y, 0.940)),
        material=powder_coat,
        name="right_handle_post",
    )
    frame.visual(
        Box((0.520, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.0, FRAME_BACK_Y, HANDLE_TOP_Z)),
        material=powder_coat,
        name="push_handle",
    )

    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, 1.030)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.515)),
    )

    caster_mounts = [
        (-0.308, -0.180),
        (0.308, -0.180),
        (-0.308, 0.180),
        (0.308, 0.180),
    ]
    for index, (x, y) in enumerate(caster_mounts):
        caster_name = _caster_part_name(index)
        _build_caster_part(model, caster_name, plated_steel, dark_rubber)
        model.articulation(
            _caster_joint_name(index),
            ArticulationType.FIXED,
            parent=frame,
            child=caster_name,
            origin=Origin(xyz=(x, y, 0.092)),
        )

    x_positions = _column_centers()
    z_floors = _row_floor_zs()
    for row, z_floor in enumerate(z_floors):
        for col, x_center in enumerate(x_positions):
            guide_name = _guide_part_name(row, col)
            drawer_name = _drawer_part_name(row, col)
            _build_guide_part(model, guide_name, plated_steel)
            _build_drawer_part(model, drawer_name, bin_plastic, pull_trim)

            model.articulation(
                _guide_joint_name(row, col),
                ArticulationType.FIXED,
                parent=frame,
                child=guide_name,
                origin=Origin(xyz=(x_center, FRAME_INNER_FRONT_Y, z_floor)),
            )
            model.articulation(
                _slide_joint_name(row, col),
                ArticulationType.PRISMATIC,
                parent=guide_name,
                child=drawer_name,
                origin=Origin(),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(
                    effort=30.0,
                    velocity=0.35,
                    lower=0.0,
                    upper=DRAWER_TRAVEL,
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

    frame = object_model.get_part("cart_frame")

    drawer_names = [_drawer_part_name(row, col) for row in range(DRAWER_ROWS) for col in range(DRAWER_COLS)]
    guide_names = [_guide_part_name(row, col) for row in range(DRAWER_ROWS) for col in range(DRAWER_COLS)]
    caster_names = [_caster_part_name(index) for index in range(4)]

    ctx.check(
        "cart has sixteen drawers",
        len(drawer_names) == 16 and all(object_model.get_part(name) for name in drawer_names),
        details=str(drawer_names),
    )
    ctx.check(
        "cart has sixteen guide assemblies",
        len(guide_names) == 16 and all(object_model.get_part(name) for name in guide_names),
        details=str(guide_names),
    )
    ctx.check(
        "cart has four casters",
        len(caster_names) == 4 and all(object_model.get_part(name) for name in caster_names),
        details=str(caster_names),
    )

    all_forward_slides = True
    slide_details: list[str] = []
    for row in range(DRAWER_ROWS):
        for col in range(DRAWER_COLS):
            joint = object_model.get_articulation(_slide_joint_name(row, col))
            limits = joint.motion_limits
            joint_ok = (
                joint.articulation_type == ArticulationType.PRISMATIC
                and tuple(round(v, 4) for v in joint.axis) == (0.0, -1.0, 0.0)
                and limits is not None
                and limits.lower == 0.0
                and limits.upper == DRAWER_TRAVEL
            )
            all_forward_slides = all_forward_slides and joint_ok
            if not joint_ok:
                slide_details.append(
                    f"{joint.name}: type={joint.articulation_type}, axis={joint.axis}, limits={limits}"
                )
    ctx.check(
        "all drawers use forward prismatic slides",
        all_forward_slides,
        details="; ".join(slide_details),
    )

    for row in range(DRAWER_ROWS):
        for col in range(DRAWER_COLS):
            guide = object_model.get_part(_guide_part_name(row, col))
            drawer = object_model.get_part(_drawer_part_name(row, col))
            slide = object_model.get_articulation(_slide_joint_name(row, col))

            ctx.expect_contact(
                guide,
                frame,
                contact_tol=0.001,
                name=f"{guide.name} mounts directly to the steel frame",
            )
            ctx.expect_contact(
                drawer,
                guide,
                contact_tol=0.001,
                name=f"{drawer.name} is supported by its guide rails at rest",
            )

            rest_pos = ctx.part_world_position(drawer)
            with ctx.pose({slide: DRAWER_TRAVEL}):
                ctx.expect_contact(
                    drawer,
                    guide,
                    contact_tol=0.001,
                    name=f"{drawer.name} stays supported when extended",
                )
                ctx.expect_overlap(
                    drawer,
                    guide,
                    axes="y",
                    min_overlap=0.220,
                    name=f"{drawer.name} retains insertion on its rails",
                )
                extended_pos = ctx.part_world_position(drawer)

            moved_forward = (
                rest_pos is not None
                and extended_pos is not None
                and extended_pos[1] < rest_pos[1] - 0.100
            )
            ctx.check(
                f"{drawer.name} slides forward when opened",
                moved_forward,
                details=f"rest={rest_pos}, extended={extended_pos}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
