from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    superellipse_profile,
)


DECK_LENGTH = 0.72
DECK_WIDTH = 0.17
DECK_THICKNESS = 0.035
DECK_Z = 0.095
HINGE_X = 0.300
HINGE_Z = 0.145
WHEEL_RADIUS = 0.055
WHEEL_WIDTH = 0.035
FRONT_WHEEL_X = 0.435
REAR_WHEEL_X = -0.435
WHEEL_Z = WHEEL_RADIUS
STEM_TOP_Z = 0.370
TELESCOPE_TRAVEL = 0.180


def _circle_profile(radius: float, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
    segments: int = 56,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments),
            [_circle_profile(inner_radius, segments)],
            length,
            center=True,
        ),
        name,
    )


def _scooter_wheel_meshes(prefix: str):
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.043,
            0.030,
            rim=WheelRim(inner_radius=0.029, flange_height=0.004, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.015,
                width=0.026,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.020, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.0024, window_radius=0.006),
            bore=WheelBore(style="round", diameter=0.008),
        ),
        f"{prefix}_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.041,
            tread=TireTread(style="circumferential", depth=0.0025, count=3),
            grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0014),),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
        ),
        f"{prefix}_tire",
    )
    return wheel_mesh, tire_mesh


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="childrens_kick_scooter")

    deck_blue = model.material("deck_blue", rgba=(0.05, 0.33, 0.90, 1.0))
    grip_black = model.material("grip_black", rgba=(0.015, 0.016, 0.018, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.025, 0.025, 0.028, 1.0))
    metal = model.material("brushed_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    accent = model.material("lime_accent", rgba=(0.52, 0.86, 0.12, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.10, 0.11, 0.12, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                superellipse_profile(DECK_LENGTH, DECK_WIDTH, exponent=2.7, segments=80),
                DECK_THICKNESS,
                center=True,
            ),
            "oval_deck",
        ),
        origin=Origin(xyz=(0.0, 0.0, DECK_Z)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                superellipse_profile(0.55, 0.115, exponent=2.6, segments=72),
                0.002,
                center=True,
            ),
            "deck_grip_pad",
        ),
        origin=Origin(xyz=(-0.020, 0.0, DECK_Z + DECK_THICKNESS / 2.0 + 0.001)),
        material=grip_black,
        name="grip_pad",
    )
    # Fork side plates are fixed to the deck and flank each wheel with real clearance.
    for x_sign, prefix in ((1.0, "front"), (-1.0, "rear")):
        fork_x = x_sign * 0.395
        for y_sign in (-1.0, 1.0):
            deck.visual(
                Box((0.140, 0.012, 0.060)),
                origin=Origin(xyz=(fork_x, y_sign * 0.053, 0.074)),
                material=deck_blue,
                name=f"{prefix}_fork_{'a' if y_sign < 0 else 'b'}",
            )
        deck.visual(
            Box((0.050, 0.115, 0.018)),
            origin=Origin(xyz=(x_sign * 0.350, 0.0, 0.069)),
            material=deck_blue,
            name=f"{prefix}_cross_bridge",
        )

    # Fold-hinge clevis fixed to the front of the deck.
    for y_sign in (-1.0, 1.0):
        deck.visual(
            Box((0.070, 0.018, 0.045)),
            origin=Origin(xyz=(HINGE_X, y_sign * 0.058, HINGE_Z)),
            material=dark_hardware,
            name=f"hinge_cheek_{'a' if y_sign < 0 else 'b'}",
        )
    deck.visual(
        Box((0.080, 0.118, 0.010)),
        origin=Origin(xyz=(HINGE_X - 0.012, 0.0, DECK_Z + DECK_THICKNESS / 2.0 + 0.005)),
        material=dark_hardware,
        name="hinge_base_plate",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.019, length=0.098),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_hardware,
        name="fold_barrel",
    )
    stem.visual(
        Box((0.040, 0.045, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_hardware,
        name="lower_yoke",
    )
    stem.visual(
        _tube_mesh(outer_radius=0.021, inner_radius=0.016, length=0.370, name="outer_stem_sleeve"),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=metal,
        name="outer_sleeve",
    )
    stem.visual(
        _tube_mesh(outer_radius=0.026, inner_radius=0.021, length=0.038, name="height_clamp_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.356)),
        material=accent,
        name="clamp_collar",
    )
    stem.visual(
        Box((0.018, 0.060, 0.018)),
        origin=Origin(xyz=(0.0, 0.047, 0.356)),
        material=accent,
        name="clamp_lever",
    )

    inner_column = model.part("inner_column")
    inner_column.visual(
        Cylinder(radius=0.012, length=0.760),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=metal,
        name="inner_tube",
    )
    inner_column.visual(
        Cylinder(radius=0.014, length=0.360),
        origin=Origin(xyz=(0.0, 0.0, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="handlebar",
    )
    for y_sign, suffix in ((-1.0, "a"), (1.0, "b")):
        inner_column.visual(
            Cylinder(radius=0.018, length=0.095),
            origin=Origin(xyz=(0.0, y_sign * 0.205, 0.420), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber_black,
            name=f"grip_{suffix}",
        )

    wheel_material = model.material("wheel_lime", rgba=(0.60, 0.88, 0.18, 1.0))
    wheel_origin = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    for wheel_name in ("front_wheel", "rear_wheel"):
        wheel = model.part(wheel_name)
        hub_mesh, tire_mesh = _scooter_wheel_meshes(wheel_name)
        wheel.visual(hub_mesh, origin=wheel_origin, material=wheel_material, name="hub")
        wheel.visual(tire_mesh, origin=wheel_origin, material=rubber_black, name="tire")
        wheel.visual(
            Cylinder(radius=0.006, length=0.094),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_hardware,
            name="axle_pin",
        )

    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=stem,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=1.5, lower=-1.45, upper=0.0),
    )
    model.articulation(
        "telescope_slide",
        ArticulationType.PRISMATIC,
        parent=stem,
        child=inner_column,
        origin=Origin(xyz=(0.0, 0.0, STEM_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=TELESCOPE_TRAVEL),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child="front_wheel",
        origin=Origin(xyz=(FRONT_WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child="rear_wheel",
        origin=Origin(xyz=(REAR_WHEEL_X, 0.0, WHEEL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    stem = object_model.get_part("stem")
    inner_column = object_model.get_part("inner_column")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    fold_hinge = object_model.get_articulation("fold_hinge")
    telescope_slide = object_model.get_articulation("telescope_slide")
    front_axle = object_model.get_articulation("front_axle")
    rear_axle = object_model.get_articulation("rear_axle")

    ctx.check(
        "primary_mechanisms_present",
        all((deck, stem, inner_column, front_wheel, rear_wheel, fold_hinge, telescope_slide, front_axle, rear_axle)),
        "Expected deck, folding stem, telescoping column, and two axle joints.",
    )
    ctx.check(
        "fold_hinge_is_revolute",
        fold_hinge.articulation_type == ArticulationType.REVOLUTE,
        f"fold_hinge type={fold_hinge.articulation_type}",
    )
    ctx.check(
        "telescope_is_prismatic",
        telescope_slide.articulation_type == ArticulationType.PRISMATIC,
        f"telescope_slide type={telescope_slide.articulation_type}",
    )
    ctx.check(
        "wheel_axles_are_continuous",
        front_axle.articulation_type == ArticulationType.CONTINUOUS
        and rear_axle.articulation_type == ArticulationType.CONTINUOUS,
        f"front={front_axle.articulation_type}, rear={rear_axle.articulation_type}",
    )
    ctx.allow_overlap(
        inner_column,
        stem,
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        reason=(
            "The telescoping inner tube is intentionally retained inside the outer sleeve; "
            "the fit is proved by the centered-within and retained-insertion checks."
        ),
    )
    ctx.allow_overlap(
        inner_column,
        stem,
        elem_a="inner_tube",
        elem_b="clamp_collar",
        reason=(
            "The adjustable clamp collar surrounds the telescoping tube at the sleeve mouth; "
            "the nested pass-through is a local captured fit."
        ),
    )

    ctx.expect_origin_gap(front_wheel, rear_wheel, axis="x", min_gap=0.80, name="wheels_sit_at_deck_ends")
    ctx.expect_within(
        inner_column,
        stem,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="inner_column_centered_in_sleeve",
    )
    ctx.expect_overlap(
        inner_column,
        stem,
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.20,
        name="collapsed_column_retained_in_sleeve",
    )
    ctx.expect_within(
        inner_column,
        stem,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="clamp_collar",
        margin=0.0,
        name="inner_column_passes_through_clamp",
    )
    ctx.expect_overlap(
        inner_column,
        stem,
        axes="z",
        elem_a="inner_tube",
        elem_b="clamp_collar",
        min_overlap=0.030,
        name="clamp_collar_encircles_inner_column",
    )

    rest_pos = ctx.part_world_position(inner_column)
    with ctx.pose({telescope_slide: TELESCOPE_TRAVEL}):
        ctx.expect_overlap(
            inner_column,
            stem,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.12,
            name="extended_column_still_retained",
        )
        extended_pos = ctx.part_world_position(inner_column)

    ctx.check(
        "telescope_extends_upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _aabb_center_zx(aabb):
        mins, maxs = aabb
        return ((float(mins[0]) + float(maxs[0])) * 0.5, (float(mins[2]) + float(maxs[2])) * 0.5)

    handlebar_rest = ctx.part_element_world_aabb(inner_column, elem="handlebar")
    with ctx.pose({fold_hinge: -1.20}):
        handlebar_folded = ctx.part_element_world_aabb(inner_column, elem="handlebar")

    rest_x, rest_z = _aabb_center_zx(handlebar_rest)
    folded_x, folded_z = _aabb_center_zx(handlebar_folded)
    ctx.check(
        "fold_hinge_lays_stem_back",
        folded_x < rest_x - 0.30 and folded_z < rest_z - 0.25,
        details=f"rest={(rest_x, rest_z)}, folded={(folded_x, folded_z)}",
    )

    return ctx.report()


object_model = build_object_model()
