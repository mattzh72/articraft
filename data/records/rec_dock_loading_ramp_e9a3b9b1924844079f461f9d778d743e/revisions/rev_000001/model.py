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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _translated_profile(
    profile: list[tuple[float, float]], dx: float, dy: float
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _build_tapered_nose(
    *,
    length: float,
    width: float,
    inner_thickness: float,
    outer_thickness: float,
) -> MeshGeometry:
    geometry = MeshGeometry()
    half_length = length * 0.5
    vertices = [
        (-half_length, 0.0, 0.0),
        (half_length, 0.0, 0.0),
        (half_length, width, 0.0),
        (-half_length, width, 0.0),
        (-half_length, 0.0, -inner_thickness),
        (half_length, 0.0, -inner_thickness),
        (half_length, width, -outer_thickness),
        (-half_length, width, -outer_thickness),
    ]
    ids = [geometry.add_vertex(*vertex) for vertex in vertices]

    _add_quad(geometry, ids[0], ids[1], ids[2], ids[3])
    _add_quad(geometry, ids[4], ids[7], ids[6], ids[5])
    _add_quad(geometry, ids[0], ids[4], ids[5], ids[1])
    _add_quad(geometry, ids[1], ids[5], ids[6], ids[2])
    _add_quad(geometry, ids[2], ids[6], ids[7], ids[3])
    _add_quad(geometry, ids[3], ids[7], ids[4], ids[0])
    return geometry


def _aabb_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][1] + aabb[1][1]) * 0.5


def _aabb_top_z(aabb) -> float | None:
    if aabb is None:
        return None
    return aabb[1][2]


def _hinge_segment_centers(
    *,
    start_x: float,
    deck_segment: float,
    child_segment: float,
) -> tuple[list[float], list[float]]:
    deck_centers: list[float] = []
    child_centers: list[float] = []
    cursor = start_x
    for segment_index in range(7):
        segment_length = deck_segment if segment_index % 2 == 0 else child_segment
        center = cursor + (segment_length * 0.5)
        if segment_index % 2 == 0:
            deck_centers.append(center)
        else:
            child_centers.append(center)
        cursor += segment_length
    return deck_centers, child_centers


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_folding_dock_plate")

    aluminum = model.material("aluminum", rgba=(0.80, 0.82, 0.84, 1.0))
    aluminum_shadow = model.material("aluminum_shadow", rgba=(0.68, 0.71, 0.74, 1.0))
    hinge_steel = model.material("hinge_steel", rgba=(0.41, 0.44, 0.48, 1.0))

    deck_length = 1.50
    deck_width = 0.54
    wing_length = 1.48
    wing_width = 0.17
    wing_nose = 0.03
    skin_thickness = 0.008
    hinge_radius = 0.007
    hinge_axis_height = hinge_radius
    hinge_axis_offset = (deck_width * 0.5) + 0.011
    deck_segment_length = 0.23
    child_segment_length = 0.17

    deck_knuckle_centers, child_knuckle_centers = _hinge_segment_centers(
        start_x=-0.715,
        deck_segment=deck_segment_length,
        child_segment=child_segment_length,
    )

    deck_surface_geometry = ExtrudeWithHolesGeometry(
        rounded_rect_profile(deck_length, deck_width, 0.024, corner_segments=8),
        [
            _translated_profile(
                rounded_rect_profile(0.16, 0.038, 0.014, corner_segments=6),
                -0.42,
                0.0,
            ),
            _translated_profile(
                rounded_rect_profile(0.16, 0.038, 0.014, corner_segments=6),
                0.42,
                0.0,
            ),
        ],
        skin_thickness,
        cap=True,
        center=True,
        closed=True,
    )
    deck_surface_mesh = mesh_from_geometry(deck_surface_geometry, "dock_plate_deck_surface")
    wing_nose_mesh = mesh_from_geometry(
        _build_tapered_nose(
            length=wing_length,
            width=wing_nose,
            inner_thickness=skin_thickness,
            outer_thickness=0.0015,
        ),
        "dock_plate_wing_nose",
    )

    deck = model.part("deck")
    deck.visual(
        deck_surface_mesh,
        origin=Origin(xyz=(0.0, 0.0, -(skin_thickness * 0.5))),
        material=aluminum,
        name="deck_surface",
    )
    deck.visual(
        Box((1.44, 0.016, 0.009)),
        origin=Origin(xyz=(0.0, hinge_axis_offset - 0.015, 0.0035)),
        material=aluminum_shadow,
        name="left_hinge_leaf",
    )
    deck.visual(
        Box((1.44, 0.016, 0.009)),
        origin=Origin(xyz=(0.0, -hinge_axis_offset + 0.015, 0.0035)),
        material=aluminum_shadow,
        name="right_hinge_leaf",
    )
    for index, center_x in enumerate(deck_knuckle_centers):
        deck.visual(
            Cylinder(radius=hinge_radius, length=deck_segment_length),
            origin=Origin(
                xyz=(center_x, hinge_axis_offset, hinge_axis_height),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name=f"left_hinge_knuckle_{index}",
        )
        deck.visual(
            Cylinder(radius=hinge_radius, length=deck_segment_length),
            origin=Origin(
                xyz=(center_x, -hinge_axis_offset, hinge_axis_height),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name=f"right_hinge_knuckle_{index}",
        )
    deck.visual(
        Box((1.28, 0.10, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.024)),
        material=aluminum_shadow,
        name="center_stringer",
    )
    deck.visual(
        Box((1.16, 0.070, 0.026)),
        origin=Origin(xyz=(0.0, 0.14, -0.021)),
        material=aluminum_shadow,
        name="left_stringer",
    )
    deck.visual(
        Box((1.16, 0.070, 0.026)),
        origin=Origin(xyz=(0.0, -0.14, -0.021)),
        material=aluminum_shadow,
        name="right_stringer",
    )
    deck.visual(
        Box((0.14, 0.38, 0.018)),
        origin=Origin(xyz=(-0.56, 0.0, -0.017)),
        material=aluminum_shadow,
        name="rear_crossmember",
    )
    deck.visual(
        Box((0.14, 0.38, 0.018)),
        origin=Origin(xyz=(0.56, 0.0, -0.017)),
        material=aluminum_shadow,
        name="front_crossmember",
    )
    deck.inertial = Inertial.from_geometry(
        Box((deck_length, 0.62, 0.050)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
    )

    left_lip = model.part("left_lip")
    left_lip.visual(
        Box((wing_length, wing_width - wing_nose, skin_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                (wing_width - wing_nose) * 0.5,
                -(hinge_axis_height + (skin_thickness * 0.5)),
            )
        ),
        material=aluminum,
        name="panel_main",
    )
    left_lip.visual(
        wing_nose_mesh,
        origin=Origin(xyz=(0.0, wing_width - wing_nose, -hinge_axis_height)),
        material=aluminum,
        name="panel_nose",
    )
    left_lip.visual(
        Box((1.44, 0.015, 0.009)),
        origin=Origin(xyz=(0.0, 0.0145, -0.0045)),
        material=aluminum_shadow,
        name="hinge_leaf",
    )
    for index, center_x in enumerate(child_knuckle_centers):
        left_lip.visual(
            Cylinder(radius=hinge_radius, length=child_segment_length),
            origin=Origin(
                xyz=(center_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name=f"knuckle_{index}",
        )
    left_lip.inertial = Inertial.from_geometry(
        Box((wing_length, wing_width, 0.028)),
        mass=4.5,
        origin=Origin(xyz=(0.0, wing_width * 0.5, -0.014)),
    )

    right_lip = model.part("right_lip")
    right_lip.visual(
        Box((wing_length, wing_width - wing_nose, skin_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -((wing_width - wing_nose) * 0.5),
                -(hinge_axis_height + (skin_thickness * 0.5)),
            )
        ),
        material=aluminum,
        name="panel_main",
    )
    right_lip.visual(
        wing_nose_mesh,
        origin=Origin(
            xyz=(0.0, -(wing_width - wing_nose), -hinge_axis_height),
            rpy=(0.0, 0.0, math.pi),
        ),
        material=aluminum,
        name="panel_nose",
    )
    right_lip.visual(
        Box((1.44, 0.015, 0.009)),
        origin=Origin(xyz=(0.0, -0.0145, -0.0045)),
        material=aluminum_shadow,
        name="hinge_leaf",
    )
    for index, center_x in enumerate(child_knuckle_centers):
        right_lip.visual(
            Cylinder(radius=hinge_radius, length=child_segment_length),
            origin=Origin(
                xyz=(center_x, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=hinge_steel,
            name=f"knuckle_{index}",
        )
    right_lip.inertial = Inertial.from_geometry(
        Box((wing_length, wing_width, 0.028)),
        mass=4.5,
        origin=Origin(xyz=(0.0, -(wing_width * 0.5), -0.014)),
    )

    model.articulation(
        "deck_to_left_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_lip,
        origin=Origin(xyz=(0.0, hinge_axis_offset, hinge_axis_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.pi,
        ),
    )
    model.articulation(
        "deck_to_right_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_lip,
        origin=Origin(xyz=(0.0, -hinge_axis_offset, hinge_axis_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    left_lip = object_model.get_part("left_lip")
    right_lip = object_model.get_part("right_lip")
    left_hinge = object_model.get_articulation("deck_to_left_lip")
    right_hinge = object_model.get_articulation("deck_to_right_lip")

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

    with ctx.pose({left_hinge: 0.0, right_hinge: 0.0}):
        ctx.expect_contact(left_lip, deck, name="left lip is hinge-mounted to the deck")
        ctx.expect_contact(right_lip, deck, name="right lip is hinge-mounted to the deck")
        ctx.expect_gap(
            left_lip,
            deck,
            axis="y",
            min_gap=0.009,
            max_gap=0.013,
            positive_elem="panel_main",
            negative_elem="deck_surface",
            name="left lip opens from the deck edge with only hinge clearance",
        )
        ctx.expect_gap(
            deck,
            right_lip,
            axis="y",
            min_gap=0.009,
            max_gap=0.013,
            positive_elem="deck_surface",
            negative_elem="panel_main",
            name="right lip opens from the deck edge with only hinge clearance",
        )
        ctx.expect_overlap(
            left_lip,
            deck,
            axes="x",
            min_overlap=1.40,
            elem_a="panel_main",
            elem_b="deck_surface",
            name="left lip spans nearly the full deck length",
        )
        ctx.expect_overlap(
            right_lip,
            deck,
            axes="x",
            min_overlap=1.40,
            elem_a="panel_main",
            elem_b="deck_surface",
            name="right lip spans nearly the full deck length",
        )

        deck_surface_aabb = ctx.part_element_world_aabb(deck, elem="deck_surface")
        left_surface_aabb = ctx.part_element_world_aabb(left_lip, elem="panel_main")
        right_surface_aabb = ctx.part_element_world_aabb(right_lip, elem="panel_main")
        ctx.check(
            "left lip top surface aligns with deck top",
            deck_surface_aabb is not None
            and left_surface_aabb is not None
            and abs(_aabb_top_z(deck_surface_aabb) - _aabb_top_z(left_surface_aabb)) <= 5e-4,
            details=f"deck={deck_surface_aabb}, left={left_surface_aabb}",
        )
        ctx.check(
            "right lip top surface aligns with deck top",
            deck_surface_aabb is not None
            and right_surface_aabb is not None
            and abs(_aabb_top_z(deck_surface_aabb) - _aabb_top_z(right_surface_aabb)) <= 5e-4,
            details=f"deck={deck_surface_aabb}, right={right_surface_aabb}",
        )

    left_open_aabb = ctx.part_element_world_aabb(left_lip, elem="panel_main")
    right_open_aabb = ctx.part_element_world_aabb(right_lip, elem="panel_main")

    with ctx.pose({left_hinge: math.pi, right_hinge: math.pi}):
        ctx.expect_gap(
            left_lip,
            deck,
            axis="z",
            min_gap=0.013,
            max_gap=0.0155,
            positive_elem="panel_main",
            negative_elem="deck_surface",
            name="left lip folds flat just above the deck for storage",
        )
        ctx.expect_gap(
            right_lip,
            deck,
            axis="z",
            min_gap=0.013,
            max_gap=0.0155,
            positive_elem="panel_main",
            negative_elem="deck_surface",
            name="right lip folds flat just above the deck for storage",
        )
        ctx.expect_overlap(
            left_lip,
            deck,
            axes="xy",
            min_overlap=0.12,
            elem_a="panel_main",
            elem_b="deck_surface",
            name="left lip nests over the deck footprint in storage",
        )
        ctx.expect_overlap(
            right_lip,
            deck,
            axes="xy",
            min_overlap=0.12,
            elem_a="panel_main",
            elem_b="deck_surface",
            name="right lip nests over the deck footprint in storage",
        )
        ctx.expect_gap(
            left_lip,
            right_lip,
            axis="y",
            min_gap=0.18,
            positive_elem="panel_main",
            negative_elem="panel_main",
            name="stored lips remain separated over the center deck",
        )

        left_closed_aabb = ctx.part_element_world_aabb(left_lip, elem="panel_main")
        right_closed_aabb = ctx.part_element_world_aabb(right_lip, elem="panel_main")
        left_open_center_y = _aabb_center_y(left_open_aabb)
        right_open_center_y = _aabb_center_y(right_open_aabb)
        left_closed_center_y = _aabb_center_y(left_closed_aabb)
        right_closed_center_y = _aabb_center_y(right_closed_aabb)
        ctx.check(
            "left lip folds inward over the deck",
            left_open_center_y is not None
            and left_closed_center_y is not None
            and left_closed_center_y < left_open_center_y - 0.10,
            details=f"open_center_y={left_open_center_y}, closed_center_y={left_closed_center_y}",
        )
        ctx.check(
            "right lip folds inward over the deck",
            right_open_center_y is not None
            and right_closed_center_y is not None
            and right_closed_center_y > right_open_center_y + 0.10,
            details=f"open_center_y={right_open_center_y}, closed_center_y={right_closed_center_y}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
