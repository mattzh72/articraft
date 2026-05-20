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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


FRAME_OUTER = (0.74, 0.59)
FRAME_INNER = (0.60, 0.45)
FRAME_HEIGHT = 0.08
FRAME_CORNER_RADIUS = 0.03
FRAME_INNER_RADIUS = 0.015

DECK_FLANGE_OUTER = (0.84, 0.69)
DECK_FLANGE_INNER = (0.72, 0.57)
DECK_FLANGE_THICKNESS = 0.012

HINGE_Y = 0.325
HINGE_Z = 0.088

LID_OUTER = (0.80, 0.65)
LID_OUTER_RADIUS = 0.035
LID_CENTER_Y = -0.325

LID_SEAT_OUTER = (0.734, 0.52)
LID_SEAT_INNER = (0.606, 0.40)
LID_SEAT_CENTER_Y = -0.35

LID_DOME_BASE = (0.64, 0.46)
LID_DOME_MID = (0.58, 0.40)
LID_DOME_TOP = (0.50, 0.32)
LID_DOME_CENTER_Y = -0.35

DOG_X_OFFSET = 0.29
DOG_Y_OFFSET = -0.59
DOG_PIN_Z = 0.009


def _rounded_loop(
    width: float,
    depth: float,
    radius: float,
    *,
    z: float,
    center_y: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + center_y, z)
        for x, y in rounded_rect_profile(
            width,
            depth,
            radius,
            corner_segments=corner_segments,
        )
    ]


def _ring_mesh(
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    *,
    height: float,
    outer_radius: float,
    inner_radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(
                outer_size[0],
                outer_size[1],
                outer_radius,
                corner_segments=8,
            ),
            [
                rounded_rect_profile(
                    inner_size[0],
                    inner_size[1],
                    inner_radius,
                    corner_segments=8,
                )
            ],
            height,
            center=True,
        ),
        name,
    )


def _plate_mesh(
    size: tuple[float, float],
    *,
    height: float,
    radius: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(
                size[0],
                size[1],
                radius,
                corner_segments=8,
            ),
            height,
            center=True,
        ),
        name,
    )


def _make_dog_geometry(part, material) -> None:
    part.visual(
        Cylinder(radius=0.016, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=material,
        name="dog_hub",
    )
    part.visual(
        Box((0.095, 0.022, 0.008)),
        origin=Origin(xyz=(0.047, 0.0, 0.006)),
        material=material,
        name="dog_arm",
    )
    part.visual(
        Cylinder(radius=0.005, length=0.028),
        origin=Origin(xyz=(0.082, 0.0, 0.0135), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="dog_handle",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.10, 0.03, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.05, 0.0, 0.012)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_deck_hatch")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.72, 0.75, 0.77, 1.0))
    lid_paint = model.material("lid_paint", rgba=(0.91, 0.93, 0.94, 1.0))
    dog_steel = model.material("dog_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.11, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        _ring_mesh(
            DECK_FLANGE_OUTER,
            DECK_FLANGE_INNER,
            height=DECK_FLANGE_THICKNESS,
            outer_radius=0.04,
            inner_radius=0.03,
            name="deck_flange_v2",
        ),
        origin=Origin(xyz=(0.0, 0.0, DECK_FLANGE_THICKNESS / 2.0)),
        material=frame_aluminum,
        name="deck_flange",
    )
    frame.visual(
        _ring_mesh(
            FRAME_OUTER,
            FRAME_INNER,
            height=FRAME_HEIGHT,
            outer_radius=FRAME_CORNER_RADIUS,
            inner_radius=FRAME_INNER_RADIUS,
            name="frame_curb_v2",
        ),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
        material=frame_aluminum,
        name="frame_curb",
    )
    frame.inertial = Inertial.from_geometry(
        Box((DECK_FLANGE_OUTER[0], DECK_FLANGE_OUTER[1], FRAME_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        _plate_mesh(
            LID_OUTER,
            height=0.006,
            radius=LID_OUTER_RADIUS,
            name="lid_top_plate_v2",
        ),
        origin=Origin(xyz=(0.0, LID_CENTER_Y, 0.002)),
        material=lid_paint,
        name="lid_top_plate",
    )
    lid.visual(
        _ring_mesh(
            LID_SEAT_OUTER,
            LID_SEAT_INNER,
            height=0.008,
            outer_radius=0.028,
            inner_radius=0.018,
            name="lid_seat_ring_v2",
        ),
        origin=Origin(xyz=(0.0, LID_SEAT_CENTER_Y, -0.004)),
        material=gasket_black,
        name="lid_seat_ring",
    )
    lid.visual(
        Box((0.60, 0.40, 0.006)),
        origin=Origin(xyz=(0.0, LID_DOME_CENTER_Y, -0.001)),
        material=lid_paint,
        name="lid_inner_pan",
    )
    lid.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _rounded_loop(
                        LID_DOME_BASE[0],
                        LID_DOME_BASE[1],
                        0.045,
                        z=0.001,
                        center_y=LID_DOME_CENTER_Y,
                    ),
                    _rounded_loop(
                        LID_DOME_MID[0],
                        LID_DOME_MID[1],
                        0.050,
                        z=0.014,
                        center_y=LID_DOME_CENTER_Y,
                    ),
                    _rounded_loop(
                        LID_DOME_TOP[0],
                        LID_DOME_TOP[1],
                        0.055,
                        z=0.024,
                        center_y=LID_DOME_CENTER_Y,
                    ),
                ]
            ),
            "lid_dome_v2",
        ),
        material=lid_paint,
        name="lid_dome",
    )
    lid.visual(
        Box((0.79, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, -0.639, -0.008)),
        material=lid_paint,
        name="front_skirt",
    )
    lid.visual(
        Box((0.028, 0.59, 0.022)),
        origin=Origin(xyz=(-0.386, -0.31, -0.009)),
        material=lid_paint,
        name="port_skirt",
    )
    lid.visual(
        Box((0.028, 0.59, 0.022)),
        origin=Origin(xyz=(0.386, -0.31, -0.009)),
        material=lid_paint,
        name="starboard_skirt",
    )
    lid.visual(
        Box((0.56, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.006, 0.005)),
        material=frame_aluminum,
        name="lid_hinge_strap",
    )
    lid.visual(
        Box((0.052, 0.032, 0.004)),
        origin=Origin(xyz=(-DOG_X_OFFSET, DOG_Y_OFFSET, DOG_PIN_Z - 0.002)),
        material=frame_aluminum,
        name="port_dog_pad",
    )
    lid.visual(
        Box((0.052, 0.032, 0.004)),
        origin=Origin(xyz=(DOG_X_OFFSET, DOG_Y_OFFSET, DOG_PIN_Z - 0.002)),
        material=frame_aluminum,
        name="starboard_dog_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((LID_OUTER[0], LID_OUTER[1], 0.055)),
        mass=14.0,
        origin=Origin(xyz=(0.0, LID_CENTER_Y, 0.0)),
    )

    port_dog = model.part("port_latch_dog")
    _make_dog_geometry(port_dog, dog_steel)

    starboard_dog = model.part("starboard_latch_dog")
    _make_dog_geometry(starboard_dog, dog_steel)

    model.articulation(
        "frame_to_lid",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "lid_to_port_latch_dog",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=port_dog,
        origin=Origin(xyz=(-DOG_X_OFFSET, DOG_Y_OFFSET, DOG_PIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-math.radians(78.0),
            upper=0.0,
        ),
    )
    model.articulation(
        "lid_to_starboard_latch_dog",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=starboard_dog,
        origin=Origin(
            xyz=(DOG_X_OFFSET, DOG_Y_OFFSET, DOG_PIN_Z),
            rpy=(0.0, 0.0, math.pi),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    lid = object_model.get_part("lid")
    port_dog = object_model.get_part("port_latch_dog")
    starboard_dog = object_model.get_part("starboard_latch_dog")

    frame_to_lid = object_model.get_articulation("frame_to_lid")
    lid_to_port_latch_dog = object_model.get_articulation("lid_to_port_latch_dog")
    lid_to_starboard_latch_dog = object_model.get_articulation("lid_to_starboard_latch_dog")

    frame_curb = frame.get_visual("frame_curb")
    lid_seat_ring = lid.get_visual("lid_seat_ring")
    lid_inner_pan = lid.get_visual("lid_inner_pan")
    lid_dome = lid.get_visual("lid_dome")
    port_dog_pad = lid.get_visual("port_dog_pad")
    starboard_dog_pad = lid.get_visual("starboard_dog_pad")
    port_hub = port_dog.get_visual("dog_hub")
    starboard_hub = starboard_dog.get_visual("dog_hub")

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
        "lid_hinge_axis",
        tuple(frame_to_lid.axis) == (-1.0, 0.0, 0.0),
        details=f"Expected rear hinge axis along -X, got {frame_to_lid.axis!r}",
    )
    ctx.check(
        "port_dog_axis",
        tuple(lid_to_port_latch_dog.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical latch axis, got {lid_to_port_latch_dog.axis!r}",
    )
    ctx.check(
        "starboard_dog_axis",
        tuple(lid_to_starboard_latch_dog.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical latch axis, got {lid_to_starboard_latch_dog.axis!r}",
    )

    lid_limits = frame_to_lid.motion_limits
    port_limits = lid_to_port_latch_dog.motion_limits
    starboard_limits = lid_to_starboard_latch_dog.motion_limits

    ctx.check(
        "lid_hinge_range",
        lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and lid_limits.upper >= math.radians(75.0),
        details=f"Unexpected lid limits {lid_limits!r}",
    )
    ctx.check(
        "port_dog_range",
        port_limits is not None
        and port_limits.lower is not None
        and port_limits.lower <= -math.radians(70.0)
        and port_limits.upper == 0.0,
        details=f"Unexpected port latch limits {port_limits!r}",
    )
    ctx.check(
        "starboard_dog_range",
        starboard_limits is not None
        and starboard_limits.lower == 0.0
        and starboard_limits.upper is not None
        and starboard_limits.upper >= math.radians(70.0),
        details=f"Unexpected starboard latch limits {starboard_limits!r}",
    )

    frame_curb_aabb = ctx.part_element_world_aabb(frame, elem=frame_curb)
    if frame_curb_aabb is not None:
        curb_height = frame_curb_aabb[1][2] - frame_curb_aabb[0][2]
        ctx.check(
            "raised_curb_height",
            0.075 <= curb_height <= 0.085,
            details=f"Frame curb height was {curb_height:.4f} m",
        )

    dome_aabb = ctx.part_element_world_aabb(lid, elem=lid_dome)
    seat_aabb = ctx.part_element_world_aabb(lid, elem=lid_seat_ring)
    if dome_aabb is not None and seat_aabb is not None:
        dome_rise = dome_aabb[1][2] - seat_aabb[1][2]
        ctx.check(
            "lid_is_slightly_domed",
            0.015 <= dome_rise <= 0.040,
            details=f"Lid dome rise was {dome_rise:.4f} m",
        )

    with ctx.pose(
        {
            frame_to_lid: 0.0,
            lid_to_port_latch_dog: 0.0,
            lid_to_starboard_latch_dog: 0.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_no_overlap")
        ctx.expect_contact(
            lid,
            frame,
            elem_a=lid_seat_ring,
            elem_b=frame_curb,
            name="lid_seats_on_frame",
        )
        ctx.expect_within(
            lid,
            frame,
            axes="xy",
            inner_elem=lid_seat_ring,
            outer_elem=frame_curb,
            margin=0.003,
            name="seat_ring_within_curb",
        )
        ctx.expect_gap(
            lid,
            frame,
            axis="z",
            positive_elem=lid_inner_pan,
            negative_elem=frame_curb,
            min_gap=0.003,
            max_gap=0.015,
            name="inner_pan_clears_frame",
        )
        ctx.expect_overlap(
            lid,
            frame,
            axes="xy",
            min_overlap=0.58,
            name="lid_plan_overlaps_frame",
        )
        ctx.expect_contact(
            port_dog,
            lid,
            elem_a=port_hub,
            elem_b=port_dog_pad,
            name="port_dog_mount_contact",
        )
        ctx.expect_contact(
            starboard_dog,
            lid,
            elem_a=starboard_hub,
            elem_b=starboard_dog_pad,
            name="starboard_dog_mount_contact",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)
        port_rest_aabb = ctx.part_world_aabb(port_dog)
        starboard_rest_aabb = ctx.part_world_aabb(starboard_dog)

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({frame_to_lid: lid_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_open_no_floating")
            ctx.expect_gap(
                lid,
                frame,
                axis="z",
                positive_elem=lid_inner_pan,
                negative_elem=frame_curb,
                min_gap=0.12,
                name="open_lid_clears_frame",
            )
            open_lid_aabb = ctx.part_world_aabb(lid)
            if closed_lid_aabb is not None and open_lid_aabb is not None:
                ctx.check(
                    "lid_lifts_high_when_open",
                    open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.20,
                    details=(
                        f"Closed max z {closed_lid_aabb[1][2]:.4f}, "
                        f"open max z {open_lid_aabb[1][2]:.4f}"
                    ),
                )

    if port_limits is not None and port_limits.lower is not None:
        with ctx.pose({lid_to_port_latch_dog: port_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="port_dog_open_no_overlap")
            ctx.expect_contact(
                port_dog,
                lid,
                elem_a=port_hub,
                elem_b=port_dog_pad,
                name="port_dog_open_stays_pinned",
            )
            port_open_aabb = ctx.part_world_aabb(port_dog)
            if port_rest_aabb is not None and port_open_aabb is not None:
                ctx.check(
                    "port_dog_swings_forward",
                    port_open_aabb[0][1] < port_rest_aabb[0][1] - 0.04,
                    details=(
                        f"Closed min y {port_rest_aabb[0][1]:.4f}, "
                        f"open min y {port_open_aabb[0][1]:.4f}"
                    ),
                )

    if starboard_limits is not None and starboard_limits.upper is not None:
        with ctx.pose({lid_to_starboard_latch_dog: starboard_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="starboard_dog_open_no_overlap")
            ctx.expect_contact(
                starboard_dog,
                lid,
                elem_a=starboard_hub,
                elem_b=starboard_dog_pad,
                name="starboard_dog_open_stays_pinned",
            )
            starboard_open_aabb = ctx.part_world_aabb(starboard_dog)
            if starboard_rest_aabb is not None and starboard_open_aabb is not None:
                ctx.check(
                    "starboard_dog_swings_forward",
                    starboard_open_aabb[0][1] < starboard_rest_aabb[0][1] - 0.04,
                    details=(
                        f"Closed min y {starboard_rest_aabb[0][1]:.4f}, "
                        f"open min y {starboard_open_aabb[0][1]:.4f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
