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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRAME_OUTER_WIDTH = 0.88
FRAME_OUTER_LENGTH = 1.12
OPENING_WIDTH = 0.75
OPENING_LENGTH = 0.94
FRAME_DEPTH = 0.056
FLANGE_THICKNESS = 0.008
SLAT_AXIS_Z = 0.010
SLAT_GLASS_WIDTH = 0.72
SLAT_CHORD = 0.145
SLAT_GLASS_THICKNESS = 0.007
SLAT_CAP_THICKNESS = 0.018
SLAT_CAP_LENGTH = 0.015
HINGE_BLOCK_WIDTH = 0.020
HINGE_BLOCK_LENGTH = 0.034
HINGE_BLOCK_HEIGHT = 0.026
PIN_RADIUS = 0.006
PIN_LENGTH = 0.015
SLAT_POSITIONS_Y = (-0.36, -0.18, 0.0, 0.18, 0.36)
OPEN_ANGLE = 0.78


def _rectangle_profile(width: float, height: float) -> list[tuple[float, float]]:
    hw = width * 0.5
    hh = height * 0.5
    return [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]


def _slat_name(index: int) -> str:
    return f"slat_{index}"


def _hinge_name(index: int) -> str:
    return f"cassette_to_slat_{index}"


def _build_slat_part(model: ArticulatedObject, index: int, glass_material, frame_material, seal_material):
    slat = model.part(_slat_name(index))
    slat.inertial = Inertial.from_geometry(
        Box((SLAT_GLASS_WIDTH + (2.0 * PIN_LENGTH), SLAT_CHORD, HINGE_BLOCK_HEIGHT)),
        mass=4.0,
    )

    slat.visual(
        Box((SLAT_GLASS_WIDTH, SLAT_CHORD, SLAT_GLASS_THICKNESS)),
        material=glass_material,
        name="glass",
    )
    slat.visual(
        Box((SLAT_GLASS_WIDTH, SLAT_CAP_LENGTH, SLAT_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, -(SLAT_CHORD - SLAT_CAP_LENGTH) * 0.5, 0.0)),
        material=frame_material,
        name="front_cap",
    )
    slat.visual(
        Box((SLAT_GLASS_WIDTH, SLAT_CAP_LENGTH, SLAT_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, (SLAT_CHORD - SLAT_CAP_LENGTH) * 0.5, 0.0)),
        material=frame_material,
        name="rear_cap",
    )

    hinge_block_x = (SLAT_GLASS_WIDTH * 0.5) - (HINGE_BLOCK_WIDTH * 0.5)
    for sign, block_name, pin_name in (
        (-1.0, "left_hinge_block", "left_pin"),
        (1.0, "right_hinge_block", "right_pin"),
    ):
        slat.visual(
            Box((HINGE_BLOCK_WIDTH, HINGE_BLOCK_LENGTH, HINGE_BLOCK_HEIGHT)),
            origin=Origin(xyz=(sign * hinge_block_x, 0.0, 0.0)),
            material=frame_material,
            name=block_name,
        )
        slat.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(
                xyz=(sign * ((SLAT_GLASS_WIDTH * 0.5) + (PIN_LENGTH * 0.5)), 0.0, 0.0),
                rpy=(0.0, pi * 0.5, 0.0),
            ),
            material=seal_material,
            name=pin_name,
        )

    return slat


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lamella_louvre_sunroof_cassette")

    frame_material = model.material("frame_material", rgba=(0.17, 0.18, 0.20, 1.0))
    trim_material = model.material("trim_material", rgba=(0.08, 0.08, 0.09, 1.0))
    glass_material = model.material("glass_material", rgba=(0.34, 0.49, 0.58, 0.42))
    seal_material = model.material("seal_material", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("cassette_frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_OUTER_LENGTH, FRAME_DEPTH + FLANGE_THICKNESS)),
        mass=20.0,
    )

    top_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rectangle_profile(FRAME_OUTER_WIDTH, FRAME_OUTER_LENGTH),
            [_rectangle_profile(OPENING_WIDTH, OPENING_LENGTH)],
            height=FLANGE_THICKNESS,
            center=True,
        ),
        "cassette_top_flange",
    )
    frame.visual(
        top_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, (FRAME_DEPTH * 0.5) - (FLANGE_THICKNESS * 0.5))),
        material=trim_material,
        name="top_flange",
    )

    side_wall_width = (FRAME_OUTER_WIDTH - OPENING_WIDTH) * 0.5
    end_wall_length = (FRAME_OUTER_LENGTH - OPENING_LENGTH) * 0.5
    wall_z = 0.0

    frame.visual(
        Box((side_wall_width, FRAME_OUTER_LENGTH, FRAME_DEPTH)),
        origin=Origin(xyz=((FRAME_OUTER_WIDTH - side_wall_width) * 0.5, 0.0, wall_z)),
        material=frame_material,
        name="left_side_rail",
    )
    frame.visual(
        Box((side_wall_width, FRAME_OUTER_LENGTH, FRAME_DEPTH)),
        origin=Origin(xyz=(-(FRAME_OUTER_WIDTH - side_wall_width) * 0.5, 0.0, wall_z)),
        material=frame_material,
        name="right_side_rail",
    )
    frame.visual(
        Box((OPENING_WIDTH, end_wall_length, FRAME_DEPTH)),
        origin=Origin(xyz=(0.0, (FRAME_OUTER_LENGTH - end_wall_length) * 0.5, wall_z)),
        material=frame_material,
        name="front_cross_member",
    )
    frame.visual(
        Box((OPENING_WIDTH, end_wall_length, FRAME_DEPTH)),
        origin=Origin(xyz=(0.0, -(FRAME_OUTER_LENGTH - end_wall_length) * 0.5, wall_z)),
        material=frame_material,
        name="rear_cross_member",
    )

    bearing_rib_width = 0.016
    bearing_rib_height = 0.022
    bearing_rib_x = (OPENING_WIDTH * 0.5) + (bearing_rib_width * 0.5)
    frame.visual(
        Box((bearing_rib_width, OPENING_LENGTH, bearing_rib_height)),
        origin=Origin(xyz=(-bearing_rib_x, 0.0, SLAT_AXIS_Z)),
        material=seal_material,
        name="left_bearing_rib",
    )
    frame.visual(
        Box((bearing_rib_width, OPENING_LENGTH, bearing_rib_height)),
        origin=Origin(xyz=(bearing_rib_x, 0.0, SLAT_AXIS_Z)),
        material=seal_material,
        name="right_bearing_rib",
    )

    slats = [_build_slat_part(model, index + 1, glass_material, frame_material, seal_material) for index in range(5)]
    for index, (slat, slat_y) in enumerate(zip(slats, SLAT_POSITIONS_Y), start=1):
        model.articulation(
            _hinge_name(index),
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(0.0, slat_y, SLAT_AXIS_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=0.92),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cassette_frame")
    slats = [object_model.get_part(_slat_name(index)) for index in range(1, 6)]
    hinges = [object_model.get_articulation(_hinge_name(index)) for index in range(1, 6)]

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

    hinge_limits_ok = all(
        hinge.motion_limits is not None
        and hinge.axis == (1.0, 0.0, 0.0)
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper >= 0.9
        for hinge in hinges
    )
    ctx.check(
        "slat hinges share a common lateral axis and opening range",
        hinge_limits_ok,
        details=str(
            [
                {
                    "name": hinge.name,
                    "axis": hinge.axis,
                    "lower": None if hinge.motion_limits is None else hinge.motion_limits.lower,
                    "upper": None if hinge.motion_limits is None else hinge.motion_limits.upper,
                }
                for hinge in hinges
            ]
        ),
    )

    for index, slat in enumerate(slats, start=1):
        ctx.expect_contact(
            frame,
            slat,
            elem_a="left_bearing_rib",
            elem_b="left_pin",
            name=f"{slat.name} left pin seats in the left side rail",
        )
        ctx.expect_contact(
            frame,
            slat,
            elem_a="right_bearing_rib",
            elem_b="right_pin",
            name=f"{slat.name} right pin seats in the right side rail",
        )

    for previous, following in zip(slats, slats[1:]):
        ctx.expect_gap(
            following,
            previous,
            axis="y",
            positive_elem="glass",
            negative_elem="glass",
            min_gap=0.028,
            max_gap=0.042,
            name=f"{previous.name} and {following.name} keep the closed-blade ventilation gap",
        )

    with ctx.pose({hinge: OPEN_ANGLE for hinge in hinges}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open lamella pose remains collision free")

        middle_front = ctx.part_element_world_aabb(slats[2], elem="front_cap")
        middle_rear = ctx.part_element_world_aabb(slats[2], elem="rear_cap")
        ctx.check(
            "positive hinge angle lifts the rear edge of a blade",
            middle_front is not None
            and middle_rear is not None
            and middle_rear[1][2] > middle_front[1][2] + 0.08,
            details=f"front={middle_front}, rear={middle_rear}",
        )

        rear_edge_heights = []
        for slat in slats:
            rear_cap_aabb = ctx.part_element_world_aabb(slat, elem="rear_cap")
            if rear_cap_aabb is not None:
                rear_edge_heights.append(rear_cap_aabb[1][2])
        ctx.check(
            "all five blades reach the same open attitude together",
            len(rear_edge_heights) == 5 and (max(rear_edge_heights) - min(rear_edge_heights)) < 0.002,
            details=f"rear_edge_heights={rear_edge_heights}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
