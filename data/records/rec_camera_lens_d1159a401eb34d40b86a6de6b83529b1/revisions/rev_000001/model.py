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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shift_perspective_correction_lens")

    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    satin_black = model.material("satin_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.70, 0.71, 0.74, 1.0))
    engraved_metal = model.material("engraved_metal", rgba=(0.48, 0.49, 0.51, 1.0))

    def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
        return [
            (
                radius * math.cos((2.0 * math.pi * index) / segments),
                radius * math.sin((2.0 * math.pi * index) / segments),
            )
            for index in range(segments)
        ]

    def _rectangular_optical_block_mesh(
        name: str,
        *,
        size_y: float,
        size_z: float,
        depth_x: float,
        corner_radius: float,
        aperture_radius: float,
    ):
        block = ExtrudeWithHolesGeometry(
            rounded_rect_profile(size_z, size_y, corner_radius, corner_segments=8),
            [_circle_profile(aperture_radius, segments=44)],
            depth_x,
            cap=True,
            center=True,
            closed=True,
        )
        return mesh_from_geometry(block, name)

    def _bayonet_plate_mesh(
        name: str,
        *,
        outer_radius: float,
        inner_radius: float,
        thickness: float,
    ):
        lug_angles = (math.radians(20.0), math.radians(145.0), math.radians(262.0))
        lug_radius = outer_radius + 0.0045
        lug_half_span = 0.20
        outer_profile: list[tuple[float, float]] = []
        for index in range(96):
            theta = (2.0 * math.pi * index) / 96.0
            radius = outer_radius
            for lug_angle in lug_angles:
                delta = ((theta - lug_angle + math.pi) % (2.0 * math.pi)) - math.pi
                if abs(delta) <= lug_half_span:
                    blend = abs(delta) / lug_half_span
                    radius = max(radius, lug_radius - (lug_radius - outer_radius) * blend)
            outer_profile.append((radius * math.cos(theta), radius * math.sin(theta)))
        ring = ExtrudeWithHolesGeometry(
            outer_profile,
            [_circle_profile(inner_radius, segments=40)],
            thickness,
            cap=True,
            center=True,
            closed=True,
        )
        return mesh_from_geometry(ring, name)

    def _shell_barrel_mesh(
        name: str,
        *,
        outer_radius: float,
        inner_radius: float,
        length: float,
    ):
        half = length * 0.5
        outer = [
            (outer_radius * 0.97, -half),
            (outer_radius, -half * 0.35),
            (outer_radius * 0.985, half),
        ]
        inner = [
            (inner_radius, -half),
            (inner_radius, half),
        ]
        shell = LatheGeometry.from_shell_profiles(
            outer,
            inner,
            segments=64,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        )
        return mesh_from_geometry(shell, name)

    rear_barrel_shell_mesh = _shell_barrel_mesh(
        "rear_barrel_shell",
        outer_radius=0.038,
        inner_radius=0.031,
        length=0.080,
    )
    front_optical_shell_mesh = _rectangular_optical_block_mesh(
        "front_optical_shell",
        size_y=0.086,
        size_z=0.058,
        depth_x=0.054,
        corner_radius=0.008,
        aperture_radius=0.028,
    )
    inner_lens_barrel_mesh = _shell_barrel_mesh(
        "front_inner_barrel",
        outer_radius=0.028,
        inner_radius=0.024,
        length=0.034,
    )
    mount_ring_mesh = _bayonet_plate_mesh(
        "bayonet_mount_ring",
        outer_radius=0.0345,
        inner_radius=0.021,
        thickness=0.006,
    )

    rear_body = model.part("rear_body")
    rear_body.visual(
        rear_barrel_shell_mesh,
        origin=Origin(xyz=(-0.058, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="rear_barrel_shell",
    )
    rear_body.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=Origin(xyz=(-0.013, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="barrel_shoulder",
    )
    rear_body.visual(
        Box((0.020, 0.066, 0.062)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=body_black,
        name="rear_carrier",
    )
    rear_body.visual(
        Box((0.028, 0.126, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, 0.034)),
        material=satin_black,
        name="upper_rail",
    )
    rear_body.visual(
        Box((0.028, 0.126, 0.006)),
        origin=Origin(xyz=(0.016, 0.0, -0.034)),
        material=satin_black,
        name="lower_rail",
    )
    rear_body.visual(
        Box((0.024, 0.006, 0.062)),
        origin=Origin(xyz=(0.016, -0.060, 0.0)),
        material=satin_black,
        name="negative_shift_stop",
    )
    rear_body.visual(
        Box((0.024, 0.006, 0.062)),
        origin=Origin(xyz=(0.016, 0.060, 0.0)),
        material=satin_black,
        name="positive_shift_stop",
    )
    rear_body.visual(
        Box((0.020, 0.026, 0.020)),
        origin=Origin(xyz=(-0.010, 0.046, -0.019)),
        material=satin_black,
        name="lock_boss",
    )
    rear_body.visual(
        Box((0.018, 0.026, 0.004)),
        origin=Origin(xyz=(0.014, -0.046, -0.003)),
        material=engraved_metal,
        name="shift_scale_plate",
    )
    rear_body.inertial = Inertial.from_geometry(
        Box((0.170, 0.130, 0.090)),
        mass=0.78,
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
    )

    front_block = model.part("front_block")
    front_block.visual(
        front_optical_shell_mesh,
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="front_block_shell",
    )
    front_block.visual(
        Box((0.020, 0.086, 0.006)),
        origin=Origin(xyz=(0.026, 0.0, 0.028)),
        material=satin_black,
        name="upper_shoe",
    )
    front_block.visual(
        Box((0.020, 0.086, 0.006)),
        origin=Origin(xyz=(0.026, 0.0, -0.028)),
        material=satin_black,
        name="lower_shoe",
    )
    front_block.visual(
        inner_lens_barrel_mesh,
        origin=Origin(xyz=(0.046, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="inner_lens_barrel",
    )
    front_block.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(0.063, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="front_filter_ring",
    )
    front_block.inertial = Inertial.from_geometry(
        Box((0.056, 0.090, 0.062)),
        mass=0.24,
        origin=Origin(xyz=(0.041, 0.0, 0.0)),
    )

    rear_mount_plate = model.part("rear_mount_plate")
    rear_mount_plate.visual(
        mount_ring_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_metal,
        name="mount_ring",
    )
    rear_mount_plate.visual(
        Box((0.002, 0.006, 0.004)),
        origin=Origin(xyz=(0.002, 0.0, 0.036)),
        material=engraved_metal,
        name="mount_index_mark",
    )
    rear_mount_plate.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.006),
        mass=0.05,
        origin=Origin(),
    )

    shift_lock_knob = model.part("shift_lock_knob")
    shift_lock_knob.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=mount_metal,
        name="knob_stem",
    )
    shift_lock_knob.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="knob_body",
    )
    shift_lock_knob.visual(
        Box((0.004, 0.010, 0.003)),
        origin=Origin(xyz=(0.0, 0.010, 0.009)),
        material=engraved_metal,
        name="knob_grip_fin",
    )
    shift_lock_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.014),
        mass=0.02,
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_front_shift",
        ArticulationType.PRISMATIC,
        parent=rear_body,
        child=front_block,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.035,
            lower=-0.012,
            upper=0.012,
        ),
    )
    model.articulation(
        "body_to_shift_lock_knob",
        ArticulationType.REVOLUTE,
        parent=rear_body,
        child=shift_lock_knob,
        origin=Origin(xyz=(-0.010, 0.059, -0.019)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-1.2,
            upper=1.2,
        ),
    )
    model.articulation(
        "body_to_rear_mount",
        ArticulationType.FIXED,
        parent=rear_body,
        child=rear_mount_plate,
        origin=Origin(xyz=(-0.101, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_body = object_model.get_part("rear_body")
    front_block = object_model.get_part("front_block")
    rear_mount_plate = object_model.get_part("rear_mount_plate")
    shift_lock_knob = object_model.get_part("shift_lock_knob")
    shift_joint = object_model.get_articulation("body_to_front_shift")
    knob_joint = object_model.get_articulation("body_to_shift_lock_knob")

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
        "front shift uses lateral prismatic joint",
        shift_joint.articulation_type == ArticulationType.PRISMATIC
        and tuple(shift_joint.axis) == (0.0, 1.0, 0.0)
        and shift_joint.motion_limits is not None
        and shift_joint.motion_limits.lower == -0.012
        and shift_joint.motion_limits.upper == 0.012,
        details=f"type={shift_joint.articulation_type}, axis={shift_joint.axis}, limits={shift_joint.motion_limits}",
    )
    ctx.check(
        "lock knob rotates about its side stem axis",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )
    ctx.expect_contact(
        rear_mount_plate,
        rear_body,
        elem_a="mount_ring",
        elem_b="rear_barrel_shell",
        name="rear bayonet plate seats against the barrel shell",
    )
    ctx.expect_contact(
        shift_lock_knob,
        rear_body,
        elem_a="knob_stem",
        elem_b="lock_boss",
        name="lock knob stem is mounted to the side boss",
    )
    ctx.expect_origin_gap(
        shift_lock_knob,
        rear_body,
        axis="y",
        min_gap=0.055,
        max_gap=0.065,
        name="lock knob sits on the housing side",
    )
    ctx.expect_contact(
        front_block,
        rear_body,
        elem_a="upper_shoe",
        elem_b="upper_rail",
        name="upper slide shoe bears on the upper rail",
    )
    ctx.expect_contact(
        front_block,
        rear_body,
        elem_a="lower_shoe",
        elem_b="lower_rail",
        name="lower slide shoe bears on the lower rail",
    )
    ctx.expect_overlap(
        front_block,
        rear_body,
        axes="y",
        elem_a="upper_shoe",
        elem_b="upper_rail",
        min_overlap=0.080,
        name="upper rail retains the shifting front block",
    )
    ctx.expect_gap(
        front_block,
        rear_body,
        axis="x",
        positive_elem="front_block_shell",
        negative_elem="rear_carrier",
        min_gap=0.0015,
        max_gap=0.0045,
        name="front block clears the fixed rear carrier",
    )

    rest_pos = ctx.part_world_position(front_block)
    with ctx.pose({shift_joint: 0.012}):
        shifted_pos = ctx.part_world_position(front_block)
        ctx.expect_contact(
            front_block,
            rear_body,
            elem_a="upper_shoe",
            elem_b="upper_rail",
            name="upper shoe stays supported at maximum shift",
        )
        ctx.expect_contact(
            front_block,
            rear_body,
            elem_a="lower_shoe",
            elem_b="lower_rail",
            name="lower shoe stays supported at maximum shift",
        )
        ctx.expect_overlap(
            front_block,
            rear_body,
            axes="y",
            elem_a="upper_shoe",
            elem_b="upper_rail",
            min_overlap=0.080,
            name="front block retains rail engagement at maximum shift",
        )

    ctx.check(
        "front block shifts laterally without optical-axis drift",
        rest_pos is not None
        and shifted_pos is not None
        and shifted_pos[1] > rest_pos[1] + 0.010
        and abs(shifted_pos[0] - rest_pos[0]) < 1e-6
        and abs(shifted_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, shifted={shifted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
