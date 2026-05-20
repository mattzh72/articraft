from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BODY_RADIUS = 0.044
BODY_THICKNESS = 0.007
GEAR_THICKNESS = 0.004
SUN_TEETH = 18
OUTER_TEETH = 27
MODULE = 0.00135
ADDENDUM = 0.0009
DEDENDUM = 0.00105
SUN_PITCH_RADIUS = 0.5 * SUN_TEETH * MODULE
OUTER_PITCH_RADIUS = 0.5 * OUTER_TEETH * MODULE
SUN_TIP_RADIUS = SUN_PITCH_RADIUS + ADDENDUM
OUTER_TIP_RADIUS = OUTER_PITCH_RADIUS + ADDENDUM
OUTER_CENTER_RADIUS = SUN_PITCH_RADIUS + OUTER_PITCH_RADIUS
SUN_BORE_RADIUS = 0.0048
OUTER_BORE_RADIUS = 0.0095
SYNC_RATIO = SUN_TEETH / OUTER_TEETH


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _circle_profile(radius: float, segments: int) -> list[tuple[float, float]]:
    return [
        _polar_xy(radius, (2.0 * math.pi * index) / segments)
        for index in range(segments)
    ]


def _spur_gear_profile(
    *,
    teeth: int,
    pitch_radius: float,
    addendum: float,
    dedendum: float,
) -> list[tuple[float, float]]:
    root_radius = max(0.0005, pitch_radius - dedendum)
    tip_radius = pitch_radius + addendum
    tooth_pitch = (2.0 * math.pi) / teeth

    profile: list[tuple[float, float]] = []
    for tooth_index in range(teeth):
        start = tooth_index * tooth_pitch
        for radius, fraction in (
            (root_radius, 0.00),
            (root_radius, 0.12),
            (tip_radius, 0.34),
            (tip_radius, 0.66),
            (root_radius, 0.88),
        ):
            profile.append(_polar_xy(radius, start + tooth_pitch * fraction))
    return profile


def _radial_hole_profiles(
    *,
    center_radius: float,
    hole_radius: float,
    count: int,
    phase: float = 0.0,
    segments: int = 24,
) -> list[list[tuple[float, float]]]:
    profiles: list[list[tuple[float, float]]] = []
    for index in range(count):
        angle = phase + (2.0 * math.pi * index) / count
        center_x, center_y = _polar_xy(center_radius, angle)
        profiles.append(
            [
                (
                    center_x + hole_radius * math.cos(sample_angle),
                    center_y + hole_radius * math.sin(sample_angle),
                )
                for sample_angle in (
                    (2.0 * math.pi * segment) / segments for segment in range(segments)
                )
            ]
        )
    return profiles


def _gear_mesh(
    *,
    name: str,
    teeth: int,
    pitch_radius: float,
    bore_radius: float,
    relief_holes: list[list[tuple[float, float]]] | None = None,
) -> object:
    outer_profile = _spur_gear_profile(
        teeth=teeth,
        pitch_radius=pitch_radius,
        addendum=ADDENDUM,
        dedendum=DEDENDUM,
    )
    hole_profiles = [_circle_profile(bore_radius, segments=max(36, teeth * 2))]
    if relief_holes is not None:
        hole_profiles.extend(relief_holes)
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            hole_profiles,
            GEAR_THICKNESS,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gear_ring_fidget")

    body_plastic = model.material("body_plastic", rgba=(0.17, 0.23, 0.31, 1.0))
    body_shadow = model.material("body_shadow", rgba=(0.10, 0.14, 0.19, 1.0))
    sun_brass = model.material("sun_brass", rgba=(0.80, 0.66, 0.28, 1.0))
    ring_bronze = model.material("ring_bronze", rgba=(0.74, 0.49, 0.24, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.70, 0.73, 0.77, 1.0))
    index_dark = model.material("index_dark", rgba=(0.18, 0.17, 0.16, 1.0))

    sun_mesh = _gear_mesh(
        name="sun_gear_spoked_mesh_v2",
        teeth=SUN_TEETH,
        pitch_radius=SUN_PITCH_RADIUS,
        bore_radius=SUN_BORE_RADIUS,
        relief_holes=_radial_hole_profiles(
            center_radius=0.0076,
            hole_radius=0.0014,
            count=6,
            phase=math.pi / 6.0,
            segments=18,
        ),
    )
    outer_mesh = _gear_mesh(
        name="outer_ring_gear_spoked_mesh_v2",
        teeth=OUTER_TEETH,
        pitch_radius=OUTER_PITCH_RADIUS,
        bore_radius=OUTER_BORE_RADIUS,
        relief_holes=_radial_hole_profiles(
            center_radius=0.0128,
            hole_radius=0.0025,
            count=3,
            phase=math.pi / 6.0,
            segments=24,
        ),
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=BODY_RADIUS + 0.0015, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=body_shadow,
        name="body_rim",
    )
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS * 0.5)),
        material=body_plastic,
        name="body_disc",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS + 0.0008)),
        material=axle_steel,
        name="sun_bearing_collar",
    )
    body.visual(
        Cylinder(radius=0.0026, length=GEAR_THICKNESS + 0.0015),
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS + 0.5 * (GEAR_THICKNESS + 0.0015))),
        material=axle_steel,
        name="sun_axle_pin",
    )
    for gear_index in range(3):
        angle = (2.0 * math.pi * gear_index) / 3.0
        center_x, center_y = _polar_xy(OUTER_CENTER_RADIUS, angle)
        body.visual(
            Cylinder(radius=0.0074, length=0.0016),
            origin=Origin(xyz=(center_x, center_y, BODY_THICKNESS + 0.0008)),
            material=axle_steel,
            name=f"outer_bearing_collar_{gear_index}",
        )
        body.visual(
            Cylinder(radius=0.0034, length=GEAR_THICKNESS + 0.0015),
            origin=Origin(
                xyz=(
                    center_x,
                    center_y,
                    BODY_THICKNESS + 0.5 * (GEAR_THICKNESS + 0.0015),
                )
            ),
            material=axle_steel,
            name=f"outer_axle_pin_{gear_index}",
        )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_RADIUS, length=BODY_THICKNESS),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, BODY_THICKNESS * 0.5)),
    )

    sun_gear = model.part("sun_gear")
    sun_gear.visual(
        sun_mesh,
        origin=Origin(xyz=(0.0, 0.0, GEAR_THICKNESS * 0.5)),
        material=sun_brass,
        name="gear_body",
    )
    sun_gear.visual(
        Cylinder(radius=0.0013, length=0.0008),
        origin=Origin(
            xyz=(SUN_PITCH_RADIUS * 0.62, 0.0, GEAR_THICKNESS + 0.0004)
        ),
        material=index_dark,
        name="index_dot",
    )
    sun_gear.inertial = Inertial.from_geometry(
        Cylinder(radius=SUN_TIP_RADIUS, length=GEAR_THICKNESS),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, GEAR_THICKNESS * 0.5)),
    )
    model.articulation(
        "body_to_sun_gear",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=sun_gear,
        origin=Origin(
            xyz=(0.0, 0.0, BODY_THICKNESS),
            rpy=(0.0, 0.0, -math.pi / SUN_TEETH),
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=20.0,
        ),
        meta={"teeth": SUN_TEETH},
    )

    for gear_index in range(3):
        angle = (2.0 * math.pi * gear_index) / 3.0
        center_x, center_y = _polar_xy(OUTER_CENTER_RADIUS, angle)
        outer_gear = model.part(f"outer_ring_gear_{gear_index}")
        outer_gear.visual(
            outer_mesh,
            origin=Origin(xyz=(0.0, 0.0, GEAR_THICKNESS * 0.5)),
            material=ring_bronze,
            name="gear_body",
        )
        outer_gear.visual(
            Cylinder(radius=0.0013, length=0.0008),
            origin=Origin(
                xyz=(OUTER_PITCH_RADIUS * 0.70, 0.0, GEAR_THICKNESS + 0.0004)
            ),
            material=index_dark,
            name="index_dot",
        )
        outer_gear.inertial = Inertial.from_geometry(
            Cylinder(radius=OUTER_TIP_RADIUS, length=GEAR_THICKNESS),
            mass=0.014,
            origin=Origin(xyz=(0.0, 0.0, GEAR_THICKNESS * 0.5)),
        )
        model.articulation(
            f"body_to_outer_ring_gear_{gear_index}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=outer_gear,
            origin=Origin(
                xyz=(center_x, center_y, BODY_THICKNESS),
                rpy=(0.0, 0.0, angle + math.pi),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=0.10,
                velocity=20.0,
            ),
            meta={
                "teeth": OUTER_TEETH,
                "mesh_with": "sun_gear",
                "gear_ratio_to_sun": -SYNC_RATIO,
            },
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    sun_gear = object_model.get_part("sun_gear")
    outer_gears = [
        object_model.get_part(f"outer_ring_gear_{index}") for index in range(3)
    ]
    sun_joint = object_model.get_articulation("body_to_sun_gear")
    outer_joints = [
        object_model.get_articulation(f"body_to_outer_ring_gear_{index}")
        for index in range(3)
    ]

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

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple(
            (aabb[0][axis] + aabb[1][axis]) * 0.5
            for axis in range(3)
        )

    def _marker_angle(part_name: str) -> float | None:
        part = object_model.get_part(part_name)
        part_origin = ctx.part_world_position(part)
        marker_aabb = ctx.part_element_world_aabb(part, elem="index_dot")
        marker_center = _aabb_center(marker_aabb)
        if part_origin is None or marker_center is None:
            return None
        return math.atan2(
            marker_center[1] - part_origin[1],
            marker_center[0] - part_origin[0],
        )

    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _axis_is_vertical(joint_obj) -> bool:
        return tuple(round(value, 6) for value in joint_obj.axis) == (0.0, 0.0, 1.0)

    ctx.check(
        "sun_joint_axis_vertical",
        _axis_is_vertical(sun_joint),
        details=f"sun axis was {sun_joint.axis}",
    )
    for index, joint_obj in enumerate(outer_joints):
        ctx.check(
            f"outer_gear_{index}_joint_axis_vertical",
            _axis_is_vertical(joint_obj),
            details=f"outer gear {index} axis was {joint_obj.axis}",
        )

    ctx.expect_contact(sun_gear, body, elem_b="body_disc", name="sun_gear_supported")
    ctx.expect_gap(
        sun_gear,
        body,
        axis="z",
        max_gap=0.0001,
        max_penetration=0.0,
        negative_elem="body_disc",
        name="sun_gear_seated_on_disc",
    )
    ctx.expect_overlap(
        sun_gear,
        body,
        axes="xy",
        min_overlap=0.024,
        name="sun_gear_over_disc",
    )

    for index, outer_gear in enumerate(outer_gears):
        ctx.expect_contact(
            outer_gear,
            body,
            elem_b="body_disc",
            name=f"outer_gear_{index}_supported",
        )
        ctx.expect_gap(
            outer_gear,
            body,
            axis="z",
            max_gap=0.0001,
            max_penetration=0.0,
            negative_elem="body_disc",
            name=f"outer_gear_{index}_seated_on_disc",
        )
        ctx.expect_overlap(
            outer_gear,
            body,
            axes="xy",
            min_overlap=0.025,
            name=f"outer_gear_{index}_over_disc",
        )
        ctx.expect_origin_distance(
            outer_gear,
            sun_gear,
            axes="xy",
            min_dist=OUTER_CENTER_RADIUS - 0.0006,
            max_dist=OUTER_CENTER_RADIUS + 0.0006,
            name=f"outer_gear_{index}_mesh_radius",
        )

    sun_angle_rest = _marker_angle("sun_gear")
    outer_angle_rest = _marker_angle("outer_ring_gear_0")
    turn_angle = 0.90
    synced_outer_angle = -turn_angle * SYNC_RATIO
    with ctx.pose(
        {
            sun_joint: turn_angle,
            outer_joints[0]: synced_outer_angle,
            outer_joints[1]: synced_outer_angle,
            outer_joints[2]: synced_outer_angle,
        }
    ):
        ctx.fail_if_isolated_parts(name="synchronized_spin_pose_supported")
        ctx.fail_if_parts_overlap_in_current_pose(name="synchronized_spin_pose_clear")
        ctx.expect_contact(sun_gear, body, elem_b="body_disc", name="sun_contact_in_sync_pose")
        for index, outer_gear in enumerate(outer_gears):
            ctx.expect_contact(
                outer_gear,
                body,
                elem_b="body_disc",
                name=f"outer_{index}_contact_in_sync_pose",
            )

        sun_angle_pose = _marker_angle("sun_gear")
        outer_angle_pose = _marker_angle("outer_ring_gear_0")
        sun_delta = None if sun_angle_rest is None or sun_angle_pose is None else _wrap_angle(sun_angle_pose - sun_angle_rest)
        outer_delta = None if outer_angle_rest is None or outer_angle_pose is None else _wrap_angle(outer_angle_pose - outer_angle_rest)
        ctx.check(
            "sun_gear_marker_turns_with_joint",
            sun_delta is not None and abs(sun_delta - turn_angle) < 0.15,
            details=f"expected sun delta about {turn_angle}, got {sun_delta}",
        )
        ctx.check(
            "outer_gear_marker_turns_with_ratio_pose",
            outer_delta is not None and abs(outer_delta - synced_outer_angle) < 0.15,
            details=f"expected outer delta about {synced_outer_angle}, got {outer_delta}",
        )

    with ctx.pose(
        {
            sun_joint: -1.10,
            outer_joints[0]: 1.10 * SYNC_RATIO,
            outer_joints[1]: 1.10 * SYNC_RATIO,
            outer_joints[2]: 1.10 * SYNC_RATIO,
        }
    ):
        ctx.fail_if_isolated_parts(name="reverse_sync_pose_supported")
        ctx.fail_if_parts_overlap_in_current_pose(name="reverse_sync_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
