from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


ROOF_WIDTH = 1.20
ROOF_LENGTH = 2.00
ROOF_THICKNESS = 0.008
ROOF_CORNER_RADIUS = 0.16

OPENING_CENTER_Y = 0.32
ROOF_HOLE_WIDTH = 0.846
ROOF_HOLE_LENGTH = 0.860
ROOF_HOLE_CORNER_RADIUS = 0.060

FRAME_OUTER_WIDTH = 0.870
FRAME_OUTER_LENGTH = 0.884
FRAME_INNER_WIDTH = 0.804
FRAME_INNER_LENGTH = 0.818
FRAME_CORNER_RADIUS = 0.060
FRAME_THICKNESS = 0.0022
GASKET_THICKNESS = 0.0012

GLASS_WIDTH = 0.796
GLASS_LENGTH = 0.810
GLASS_THICKNESS = 0.005
GLASS_CORNER_RADIUS = 0.055
GLASS_CLOSED_BOTTOM_Z = ROOF_THICKNESS - GLASS_THICKNESS

RAIL_CENTER_X = 0.462
RAIL_CENTER_Y = -0.10
RAIL_LENGTH = 1.60
RAIL_OUTER_WIDTH = 0.050
RAIL_WALL_THICKNESS = 0.007
RAIL_HEIGHT = 0.046
RAIL_CAP_THICKNESS = 0.006

SHOE_X_OFFSET = RAIL_CENTER_X
SHOE_FRONT_Y_OFFSET = 0.315
SHOE_REAR_Y_OFFSET = -0.315
SHOE_JOINT_Z = -0.015
SLIDER_SIZE = (0.028, 0.070, 0.012)

GLASS_RETRACT_DISTANCE = 0.835
GLASS_DROP_DISTANCE = 0.022
GLASS_SLIDE_STROKE = math.sqrt(GLASS_RETRACT_DISTANCE**2 + GLASS_DROP_DISTANCE**2)
GLASS_SLIDE_AXIS = (
    0.0,
    -GLASS_RETRACT_DISTANCE / GLASS_SLIDE_STROKE,
    -GLASS_DROP_DISTANCE / GLASS_SLIDE_STROKE,
)

ROOF_HOLE_REAR_EDGE = OPENING_CENTER_Y - ROOF_HOLE_LENGTH * 0.5
FRAME_INNER_MIN_X = -FRAME_INNER_WIDTH * 0.5
FRAME_INNER_MAX_X = FRAME_INNER_WIDTH * 0.5
FRAME_INNER_MIN_Y = OPENING_CENTER_Y - FRAME_INNER_LENGTH * 0.5
FRAME_INNER_MAX_Y = OPENING_CENTER_Y + FRAME_INNER_LENGTH * 0.5


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _shift_profile(
    profile: list[tuple[float, float]], *, dx: float = 0.0, dy: float = 0.0
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rounded_panel_mesh(
    name: str,
    *,
    width: float,
    length: float,
    thickness: float,
    radius: float,
):
    return _mesh(
        name,
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, length, radius, corner_segments=10),
            thickness,
        ),
    )


def _rounded_ring_mesh(
    name: str,
    *,
    outer_width: float,
    outer_length: float,
    inner_width: float,
    inner_length: float,
    thickness: float,
    radius: float,
    center_y: float = 0.0,
):
    outer_profile = _shift_profile(
        rounded_rect_profile(outer_width, outer_length, radius, corner_segments=10),
        dy=center_y,
    )
    inner_profile = _shift_profile(
        rounded_rect_profile(
            inner_width,
            inner_length,
            max(0.001, radius - 0.004),
            corner_segments=10,
        ),
        dy=center_y,
    )
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer_profile,
            [inner_profile],
            height=thickness,
            center=True,
        ),
    )


def _build_rail_part(model: ArticulatedObject, *, name: str, side_sign: float, rail_material, liner_material):
    rail = model.part(name)
    outer_wall_center_x = side_sign * (RAIL_OUTER_WIDTH * 0.5 - RAIL_WALL_THICKNESS * 0.5)
    inner_wall_center_x = -side_sign * (RAIL_OUTER_WIDTH * 0.5 - RAIL_WALL_THICKNESS * 0.5)
    guide_cheek_center_z = -0.012
    guide_cheek_height = 0.012
    guide_cheek_width = 0.005
    guide_cheek_center_x = 0.0185

    rail.visual(
        Box((RAIL_WALL_THICKNESS, RAIL_LENGTH, 0.042)),
        origin=Origin(xyz=(outer_wall_center_x, 0.0, -0.021)),
        material=rail_material,
        name="outer_wall",
    )
    rail.visual(
        Box((RAIL_WALL_THICKNESS, RAIL_LENGTH, 0.034)),
        origin=Origin(xyz=(inner_wall_center_x, 0.0, -0.025)),
        material=rail_material,
        name="inner_wall",
    )
    rail.visual(
        Box((0.032, RAIL_LENGTH, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=liner_material,
        name="runner_deck",
    )
    rail.visual(
        Box((guide_cheek_width, RAIL_LENGTH, guide_cheek_height)),
        origin=Origin(
            xyz=(
                guide_cheek_center_x,
                0.0,
                guide_cheek_center_z,
            )
        ),
        material=liner_material,
        name="outer_guide",
    )
    rail.visual(
        Box((guide_cheek_width, RAIL_LENGTH, guide_cheek_height)),
        origin=Origin(
            xyz=(
                -guide_cheek_center_x,
                0.0,
                guide_cheek_center_z,
            )
        ),
        material=liner_material,
        name="inner_guide",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_OUTER_WIDTH + 0.020, RAIL_LENGTH, RAIL_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.023)),
    )
    return rail


def _build_shoe_part(model: ArticulatedObject, *, name: str, side_sign: float, shoe_material, bracket_material):
    shoe = model.part(name)
    shoe.visual(
        Box(SLIDER_SIZE),
        origin=Origin(),
        material=shoe_material,
        name="slider_block",
    )
    shoe.visual(
        Box((0.012, 0.024, 0.016)),
        origin=Origin(
            xyz=(
                -side_sign * 0.006,
                0.0,
                0.002,
            )
        ),
        material=bracket_material,
        name="riser",
    )
    shoe.visual(
        Box((0.060, 0.026, 0.004)),
        origin=Origin(
            xyz=(
                -side_sign * 0.034,
                0.0,
                0.002,
            )
        ),
        material=bracket_material,
        name="yoke_arm",
    )
    shoe.visual(
        Box((0.010, 0.022, 0.008)),
        origin=Origin(
            xyz=(
                -side_sign * 0.062,
                0.0,
                0.007,
            )
        ),
        material=bracket_material,
        name="inner_riser",
    )
    shoe.visual(
        Box((0.032, 0.032, 0.004)),
        origin=Origin(
            xyz=(
                -side_sign * 0.068,
                0.0,
                0.013,
            )
        ),
        material=bracket_material,
        name="glass_pad",
    )
    shoe.inertial = Inertial.from_geometry(
        Box((0.110, 0.080, 0.030)),
        mass=0.32,
        origin=Origin(xyz=(-side_sign * 0.038, 0.0, 0.006)),
    )
    return shoe


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="frameless_glass_sunroof_cassette")

    body_paint = model.material("body_paint", rgba=(0.63, 0.65, 0.69, 1.0))
    cassette_black = model.material("cassette_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rail_aluminium = model.material("rail_aluminium", rgba=(0.69, 0.71, 0.73, 1.0))
    brushed_aluminium = model.material("brushed_aluminium", rgba=(0.76, 0.78, 0.80, 1.0))
    shoe_polymer = model.material("shoe_polymer", rgba=(0.12, 0.12, 0.13, 1.0))
    bracket_steel = model.material("bracket_steel", rgba=(0.43, 0.45, 0.47, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.27, 0.31, 0.42))
    glass_frit = model.material("glass_frit", rgba=(0.05, 0.05, 0.06, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.04, 0.04, 0.05, 1.0))

    cassette_body = model.part("cassette_body")
    cassette_body.visual(
        _mesh(
            "sunroof_roof_skin",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(
                    ROOF_WIDTH,
                    ROOF_LENGTH,
                    ROOF_CORNER_RADIUS,
                    corner_segments=12,
                ),
                [
                    _shift_profile(
                        rounded_rect_profile(
                            ROOF_HOLE_WIDTH,
                            ROOF_HOLE_LENGTH,
                            ROOF_HOLE_CORNER_RADIUS,
                            corner_segments=10,
                        ),
                        dy=OPENING_CENTER_Y,
                    )
                ],
                height=ROOF_THICKNESS,
                center=True,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, ROOF_THICKNESS * 0.5)),
        material=body_paint,
        name="roof_skin",
    )
    cassette_body.visual(
        Box((0.090, 1.66, 0.040)),
        origin=Origin(xyz=(0.532, -0.12, -0.016)),
        material=cassette_black,
        name="left_trough",
    )
    cassette_body.visual(
        Box((0.090, 1.66, 0.040)),
        origin=Origin(xyz=(-0.532, -0.12, -0.016)),
        material=cassette_black,
        name="right_trough",
    )
    cassette_body.visual(
        Box((0.900, 0.090, 0.030)),
        origin=Origin(xyz=(0.0, 0.795, -0.011)),
        material=cassette_black,
        name="front_header",
    )
    cassette_body.visual(
        Box((0.300, 0.760, 0.040)),
        origin=Origin(xyz=(0.0, -0.57, -0.028)),
        material=cassette_black,
        name="rear_cartridge",
    )
    cassette_body.visual(
        Box((0.240, 0.080, 0.020)),
        origin=Origin(xyz=(0.0, -0.95, -0.012)),
        material=cassette_black,
        name="rear_crossmember",
    )
    cassette_body.inertial = Inertial.from_geometry(
        Box((ROOF_WIDTH, ROOF_LENGTH, 0.080)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    seal_frame = model.part("seal_frame")
    seal_frame.visual(
        _rounded_ring_mesh(
            "sunroof_frame_ring",
            outer_width=FRAME_OUTER_WIDTH,
            outer_length=FRAME_OUTER_LENGTH,
            inner_width=FRAME_INNER_WIDTH,
            inner_length=FRAME_INNER_LENGTH,
            thickness=FRAME_THICKNESS,
            radius=FRAME_CORNER_RADIUS,
            center_y=OPENING_CENTER_Y,
        ),
        origin=Origin(xyz=(0.0, 0.0, ROOF_THICKNESS + FRAME_THICKNESS * 0.5)),
        material=brushed_aluminium,
        name="frame_ring",
    )
    seal_frame.visual(
        _rounded_ring_mesh(
            "sunroof_gasket_ring",
            outer_width=FRAME_INNER_WIDTH + 0.010,
            outer_length=FRAME_INNER_LENGTH + 0.010,
            inner_width=FRAME_INNER_WIDTH - 0.006,
            inner_length=FRAME_INNER_LENGTH - 0.006,
            thickness=GASKET_THICKNESS,
            radius=FRAME_CORNER_RADIUS - 0.004,
            center_y=OPENING_CENTER_Y,
        ),
        origin=Origin(xyz=(0.0, 0.0, ROOF_THICKNESS + GASKET_THICKNESS * 0.5)),
        material=seal_rubber,
        name="gasket_ring",
    )
    seal_frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_OUTER_LENGTH, FRAME_THICKNESS + GASKET_THICKNESS)),
        mass=1.3,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, ROOF_THICKNESS + 0.0016)),
    )

    left_rail = _build_rail_part(
        model,
        name="left_rail",
        side_sign=1.0,
        rail_material=rail_aluminium,
        liner_material=cassette_black,
    )
    right_rail = _build_rail_part(
        model,
        name="right_rail",
        side_sign=-1.0,
        rail_material=rail_aluminium,
        liner_material=cassette_black,
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        _rounded_panel_mesh(
            "sunroof_glass_panel",
            width=GLASS_WIDTH,
            length=GLASS_LENGTH,
            thickness=GLASS_THICKNESS,
            radius=GLASS_CORNER_RADIUS,
        ),
        material=glass_tint,
        name="glass_lite",
    )
    glass_panel.visual(
        _rounded_ring_mesh(
            "sunroof_glass_frit",
            outer_width=GLASS_WIDTH,
            outer_length=GLASS_LENGTH,
            inner_width=GLASS_WIDTH - 0.050,
            inner_length=GLASS_LENGTH - 0.050,
            thickness=0.0008,
            radius=GLASS_CORNER_RADIUS,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0004)),
        material=glass_frit,
        name="glass_frit",
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((GLASS_WIDTH, GLASS_LENGTH, GLASS_THICKNESS)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, GLASS_THICKNESS * 0.5)),
    )

    front_left_shoe = _build_shoe_part(
        model,
        name="front_left_shoe",
        side_sign=1.0,
        shoe_material=shoe_polymer,
        bracket_material=bracket_steel,
    )
    front_right_shoe = _build_shoe_part(
        model,
        name="front_right_shoe",
        side_sign=-1.0,
        shoe_material=shoe_polymer,
        bracket_material=bracket_steel,
    )
    rear_left_shoe = _build_shoe_part(
        model,
        name="rear_left_shoe",
        side_sign=1.0,
        shoe_material=shoe_polymer,
        bracket_material=bracket_steel,
    )
    rear_right_shoe = _build_shoe_part(
        model,
        name="rear_right_shoe",
        side_sign=-1.0,
        shoe_material=shoe_polymer,
        bracket_material=bracket_steel,
    )

    model.articulation(
        "cassette_to_seal_frame",
        ArticulationType.FIXED,
        parent=cassette_body,
        child=seal_frame,
        origin=Origin(),
    )
    model.articulation(
        "cassette_to_left_rail",
        ArticulationType.FIXED,
        parent=cassette_body,
        child=left_rail,
        origin=Origin(xyz=(RAIL_CENTER_X, RAIL_CENTER_Y, 0.0)),
    )
    model.articulation(
        "cassette_to_right_rail",
        ArticulationType.FIXED,
        parent=cassette_body,
        child=right_rail,
        origin=Origin(xyz=(-RAIL_CENTER_X, RAIL_CENTER_Y, 0.0)),
    )
    model.articulation(
        "glass_slide",
        ArticulationType.PRISMATIC,
        parent=cassette_body,
        child=glass_panel,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, GLASS_CLOSED_BOTTOM_Z)),
        axis=GLASS_SLIDE_AXIS,
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=GLASS_SLIDE_STROKE,
        ),
    )
    model.articulation(
        "glass_to_front_left_shoe",
        ArticulationType.FIXED,
        parent=glass_panel,
        child=front_left_shoe,
        origin=Origin(xyz=(SHOE_X_OFFSET, SHOE_FRONT_Y_OFFSET, SHOE_JOINT_Z)),
    )
    model.articulation(
        "glass_to_front_right_shoe",
        ArticulationType.FIXED,
        parent=glass_panel,
        child=front_right_shoe,
        origin=Origin(xyz=(-SHOE_X_OFFSET, SHOE_FRONT_Y_OFFSET, SHOE_JOINT_Z)),
    )
    model.articulation(
        "glass_to_rear_left_shoe",
        ArticulationType.FIXED,
        parent=glass_panel,
        child=rear_left_shoe,
        origin=Origin(xyz=(SHOE_X_OFFSET, SHOE_REAR_Y_OFFSET, SHOE_JOINT_Z)),
    )
    model.articulation(
        "glass_to_rear_right_shoe",
        ArticulationType.FIXED,
        parent=glass_panel,
        child=rear_right_shoe,
        origin=Origin(xyz=(-SHOE_X_OFFSET, SHOE_REAR_Y_OFFSET, SHOE_JOINT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    cassette_body = object_model.get_part("cassette_body")
    seal_frame = object_model.get_part("seal_frame")
    left_rail = object_model.get_part("left_rail")
    right_rail = object_model.get_part("right_rail")
    glass_panel = object_model.get_part("glass_panel")
    front_left_shoe = object_model.get_part("front_left_shoe")
    front_right_shoe = object_model.get_part("front_right_shoe")
    rear_left_shoe = object_model.get_part("rear_left_shoe")
    rear_right_shoe = object_model.get_part("rear_right_shoe")
    glass_slide = object_model.get_articulation("glass_slide")

    def require_part_aabb(part):
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            ctx.fail(f"{part.name}_aabb_available", f"Missing world AABB for {part.name}.")
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return aabb

    def require_elem_aabb(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            ctx.fail(
                f"{part.name}_{elem_name}_aabb_available",
                f"Missing world AABB for {part.name}:{elem_name}.",
            )
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return aabb

    ctx.expect_contact(seal_frame, cassette_body)
    ctx.expect_contact(left_rail, cassette_body)
    ctx.expect_contact(right_rail, cassette_body)

    shoe_to_rail = [
        (front_left_shoe, left_rail),
        (rear_left_shoe, left_rail),
        (front_right_shoe, right_rail),
        (rear_right_shoe, right_rail),
    ]
    for shoe, rail in shoe_to_rail:
        ctx.expect_contact(glass_panel, shoe, elem_a="glass_lite", elem_b="glass_pad")
        ctx.expect_contact(shoe, rail, elem_a="slider_block", elem_b="runner_deck")
        ctx.expect_within(shoe, rail, axes="xy", inner_elem="slider_block", margin=0.0035)

    axis = glass_slide.axis
    limits = glass_slide.motion_limits
    ctx.check(
        "glass slide axis is rearward and descending",
        axis[0] == 0.0 and axis[1] < -0.99 and axis[2] < -0.02,
        f"Unexpected slide axis {axis}.",
    )
    ctx.check(
        "glass slide stroke is long enough for full retraction",
        limits is not None and limits.upper is not None and limits.upper >= 0.82,
        f"Unexpected slide stroke {None if limits is None else limits.upper}.",
    )

    roof_skin_aabb = require_elem_aabb(cassette_body, "roof_skin")
    frame_ring_aabb = require_elem_aabb(seal_frame, "frame_ring")
    glass_closed_aabb = require_elem_aabb(glass_panel, "glass_lite")

    roof_top = roof_skin_aabb[1][2]
    frame_top = frame_ring_aabb[1][2]
    glass_top = glass_closed_aabb[1][2]
    ctx.check(
        "glass sits flush with roof skin",
        abs(glass_top - roof_top) <= 0.0015,
        f"Roof top {roof_top:.4f} vs glass top {glass_top:.4f}.",
    )
    ctx.check(
        "seal frame stays thin above glass",
        0.0010 <= frame_top - glass_top <= 0.0035,
        f"Frame top {frame_top:.4f} vs glass top {glass_top:.4f}.",
    )
    ctx.check(
        "glass is nested inside the frame aperture",
        glass_closed_aabb[0][0] >= FRAME_INNER_MIN_X + 0.001
        and glass_closed_aabb[1][0] <= FRAME_INNER_MAX_X - 0.001
        and glass_closed_aabb[0][1] >= FRAME_INNER_MIN_Y + 0.001
        and glass_closed_aabb[1][1] <= FRAME_INNER_MAX_Y - 0.001,
        (
            "Glass AABB "
            f"{glass_closed_aabb} exceeds frame aperture "
            f"x[{FRAME_INNER_MIN_X:.3f},{FRAME_INNER_MAX_X:.3f}] "
            f"y[{FRAME_INNER_MIN_Y:.3f},{FRAME_INNER_MAX_Y:.3f}]."
        ),
    )

    with ctx.pose({glass_slide: GLASS_SLIDE_STROKE}):
        glass_open_aabb = require_elem_aabb(glass_panel, "glass_lite")
        ctx.check(
            "glass retracts fully behind the roof cutout",
            glass_open_aabb[1][1] <= ROOF_HOLE_REAR_EDGE + 0.002,
            (
                f"Open glass front edge {glass_open_aabb[1][1]:.4f} "
                f"should be behind roof-cutout rear edge {ROOF_HOLE_REAR_EDGE:.4f}."
            ),
        )
        ctx.expect_gap(
            cassette_body,
            glass_panel,
            axis="z",
            positive_elem="roof_skin",
            negative_elem="glass_lite",
            min_gap=0.010,
            max_gap=0.026,
        )

        glass_open_center_y = (glass_open_aabb[0][1] + glass_open_aabb[1][1]) * 0.5
        glass_closed_center_y = (glass_closed_aabb[0][1] + glass_closed_aabb[1][1]) * 0.5
        ctx.check(
            "glass moves substantially rearward",
            glass_open_center_y <= glass_closed_center_y - 0.80,
            (
                f"Closed center y {glass_closed_center_y:.4f} vs "
                f"open center y {glass_open_center_y:.4f}."
            ),
        )

        for shoe, rail in shoe_to_rail:
            slider_aabb = require_elem_aabb(shoe, "slider_block")
            rail_aabb = require_part_aabb(rail)
            ctx.expect_within(shoe, rail, axes="xy", inner_elem="slider_block", margin=0.0035)
            ctx.check(
                f"{shoe.name} slider stays inside rail depth",
                slider_aabb[0][2] >= rail_aabb[0][2] - 0.001
                and slider_aabb[1][2] <= rail_aabb[1][2] + 0.001,
                f"Slider AABB {slider_aabb} vs rail AABB {rail_aabb}.",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
