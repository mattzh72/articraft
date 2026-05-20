from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib

_REAL_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _REAL_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd
pathlib.os.getcwd = _safe_getcwd
try:
    os.chdir(_safe_getcwd())
except Exception:
    pass

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(geometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _shell_mesh(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
):
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _cylindrical_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    start_z: float,
    end_z: float,
    segments: int = 72,
):
    return _shell_mesh(
        [(outer_radius, start_z), (outer_radius, end_z)],
        [(inner_radius, start_z), (inner_radius, end_z)],
        segments=segments,
    )


def _focus_sleeve_mesh():
    return _shell_mesh(
        [
            (0.0558, -0.019),
            (0.0562, -0.015),
            (0.0562, 0.015),
            (0.0558, 0.019),
        ],
        [
            (0.0542, -0.019),
            (0.0542, 0.019),
        ],
    )


def _iris_sleeve_mesh():
    return _shell_mesh(
        [
            (0.0508, -0.014),
            (0.0513, -0.011),
            (0.0513, 0.011),
            (0.0508, 0.014),
        ],
        [
            (0.0490, -0.014),
            (0.0490, 0.014),
        ],
    )


def _position_delta(a: tuple[float, float, float] | None, b: tuple[float, float, float] | None) -> float:
    if a is None or b is None:
        return float("inf")
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cine_lens", assets=ASSETS)

    anodized_black = model.material("anodized_black", rgba=(0.12, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    matte_charcoal = model.material("matte_charcoal", rgba=(0.08, 0.08, 0.09, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.25, 0.27, 1.0))
    engraved_white = model.material("engraved_white", rgba=(0.87, 0.87, 0.84, 1.0))
    smoky_glass = model.material("smoky_glass", rgba=(0.22, 0.26, 0.30, 0.58))

    barrel_body = model.part("barrel_body")
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0420,
                inner_radius=0.0310,
                start_z=0.000,
                end_z=0.028,
            ),
            "rear_mount_shell.obj",
        ),
        material=gunmetal,
        name="rear_mount_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0485,
                inner_radius=0.0330,
                start_z=0.028,
                end_z=0.064,
            ),
            "iris_seat_shell.obj",
        ),
        material=anodized_black,
        name="iris_seat_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0505,
                inner_radius=0.0342,
                start_z=0.064,
                end_z=0.106,
            ),
            "mid_barrel_shell.obj",
        ),
        material=anodized_black,
        name="mid_barrel_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0538,
                inner_radius=0.0360,
                start_z=0.106,
                end_z=0.152,
            ),
            "focus_seat_shell.obj",
        ),
        material=satin_black,
        name="focus_seat_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _shell_mesh(
                [
                    (0.0568, 0.152),
                    (0.0578, 0.170),
                    (0.0578, 0.214),
                ],
                [
                    (0.0372, 0.152),
                    (0.0388, 0.214),
                ],
            ),
            "front_housing_shell.obj",
        ),
        material=anodized_black,
        name="front_housing_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0605,
                inner_radius=0.0378,
                start_z=0.214,
                end_z=0.234,
            ),
            "front_name_ring_shell.obj",
        ),
        material=gunmetal,
        name="front_name_ring_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0620,
                inner_radius=0.0348,
                start_z=0.234,
                end_z=0.236,
            ),
            "front_rim_shell.obj",
        ),
        material=satin_black,
        name="front_rim_shell",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0330,
                inner_radius=0.0292,
                start_z=0.038,
                end_z=0.056,
            ),
            "rear_baffle_ring.obj",
        ),
        material=matte_charcoal,
        name="rear_baffle",
    )
    barrel_body.visual(
        Cylinder(radius=0.0345, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=matte_charcoal,
        name="mid_baffle",
    )
    barrel_body.visual(
        Cylinder(radius=0.0378, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        material=matte_charcoal,
        name="front_baffle",
    )
    barrel_body.visual(
        _save_mesh(
            _cylindrical_shell(
                outer_radius=0.0378,
                inner_radius=0.0342,
                start_z=0.225,
                end_z=0.229,
            ),
            "front_retainer_ring.obj",
        ),
        material=gunmetal,
        name="front_retainer",
    )
    barrel_body.visual(
        Cylinder(radius=0.0342, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.2295)),
        material=smoky_glass,
        name="front_glass",
    )
    barrel_body.visual(
        Box((0.0014, 0.0008, 0.012)),
        origin=Origin(xyz=(0.0, 0.0489, 0.046)),
        material=engraved_white,
        name="iris_index_line",
    )
    barrel_body.visual(
        Box((0.0014, 0.0008, 0.016)),
        origin=Origin(xyz=(0.0, 0.0542, 0.129)),
        material=engraved_white,
        name="focus_index_line",
    )
    barrel_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.236),
        mass=1.85,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _save_mesh(_focus_sleeve_mesh(), "focus_sleeve.obj"),
        material=matte_charcoal,
        name="focus_sleeve",
    )
    focus_tooth_count = 56
    focus_pitch = (math.tau * 0.0592) / focus_tooth_count
    for index in range(focus_tooth_count):
        angle = index * math.tau / focus_tooth_count
        focus_ring.visual(
            Box((0.0066, focus_pitch * 0.63, 0.035)),
            origin=Origin(xyz=(0.0588, 0.0, 0.0), rpy=(0.0, 0.0, angle)),
            material=matte_charcoal,
            name=f"gear_tooth_{index:02d}",
        )
    for pad_index in range(6):
        angle = pad_index * math.tau / 6.0
        focus_ring.visual(
            Box((0.0018, 0.0065, 0.008)),
            origin=Origin(xyz=(0.0547, 0.0, 0.0), rpy=(0.0, 0.0, angle)),
            material=gunmetal,
            name=f"focus_pad_{pad_index}",
        )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.038),
        mass=0.22,
        origin=Origin(),
    )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        _save_mesh(_iris_sleeve_mesh(), "iris_sleeve.obj"),
        material=satin_black,
        name="iris_sleeve",
    )
    iris_grip_count = 36
    iris_pitch = (math.tau * 0.0521) / iris_grip_count
    for index in range(iris_grip_count):
        angle = index * math.tau / iris_grip_count
        iris_ring.visual(
            Box((0.0036, iris_pitch * 0.56, 0.026)),
            origin=Origin(xyz=(0.0522, 0.0, 0.0), rpy=(0.0, 0.0, angle)),
            material=satin_black,
            name=f"iris_grip_{index:02d}",
        )
    for pad_index in range(4):
        angle = pad_index * math.tau / 4.0
        iris_ring.visual(
            Box((0.0018, 0.0052, 0.006)),
            origin=Origin(xyz=(0.0494, 0.0, 0.0), rpy=(0.0, 0.0, angle)),
            material=gunmetal,
            name=f"iris_pad_{pad_index}",
        )
    iris_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.054, length=0.028),
        mass=0.14,
        origin=Origin(),
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel_body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-2.45, upper=2.45),
    )
    model.articulation(
        "iris_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel_body,
        child=iris_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.60, upper=0.60),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    barrel_body = object_model.get_part("barrel_body")
    focus_ring = object_model.get_part("focus_ring")
    iris_ring = object_model.get_part("iris_ring")

    focus_rotation = object_model.get_articulation("focus_rotation")
    iris_rotation = object_model.get_articulation("iris_rotation")

    rear_mount_shell = barrel_body.get_visual("rear_mount_shell")
    iris_seat_shell = barrel_body.get_visual("iris_seat_shell")
    focus_seat_shell = barrel_body.get_visual("focus_seat_shell")
    front_housing_shell = barrel_body.get_visual("front_housing_shell")
    front_rim_shell = barrel_body.get_visual("front_rim_shell")
    front_glass = barrel_body.get_visual("front_glass")

    focus_pad = focus_ring.get_visual("focus_pad_0")
    gear_tooth = focus_ring.get_visual("gear_tooth_00")
    iris_pad = iris_ring.get_visual("iris_pad_0")
    iris_grip = iris_ring.get_visual("iris_grip_00")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    focus_limits = focus_rotation.motion_limits
    iris_limits = iris_rotation.motion_limits

    ctx.check(
        "focus_rotation_axis_matches_optical_axis",
        tuple(round(v, 6) for v in focus_rotation.axis) == (0.0, 0.0, 1.0),
        f"Focus axis was {focus_rotation.axis!r}",
    )
    ctx.check(
        "iris_rotation_axis_matches_optical_axis",
        tuple(round(v, 6) for v in iris_rotation.axis) == (0.0, 0.0, 1.0),
        f"Iris axis was {iris_rotation.axis!r}",
    )
    ctx.check(
        "focus_throw_is_cine_like",
        (
            focus_limits is not None
            and focus_limits.lower is not None
            and focus_limits.upper is not None
            and (focus_limits.upper - focus_limits.lower) >= 4.5
        ),
        f"Focus limits were {focus_limits!r}",
    )
    ctx.check(
        "iris_throw_is_shorter_than_focus_throw",
        (
            focus_limits is not None
            and iris_limits is not None
            and focus_limits.lower is not None
            and focus_limits.upper is not None
            and iris_limits.lower is not None
            and iris_limits.upper is not None
            and (iris_limits.upper - iris_limits.lower) < (focus_limits.upper - focus_limits.lower)
            and (iris_limits.upper - iris_limits.lower) >= 1.0
        ),
        f"Focus limits were {focus_limits!r}; iris limits were {iris_limits!r}",
    )

    ctx.expect_contact(
        focus_ring,
        barrel_body,
        elem_a=focus_pad,
        elem_b=focus_seat_shell,
        contact_tol=0.0015,
        name="focus_ring_is_borne_by_focus_seat",
    )
    ctx.expect_contact(
        iris_ring,
        barrel_body,
        elem_a=iris_pad,
        elem_b=iris_seat_shell,
        contact_tol=0.0015,
        name="iris_ring_is_borne_by_iris_seat",
    )
    ctx.expect_origin_distance(focus_ring, barrel_body, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(iris_ring, barrel_body, axes="xy", max_dist=0.001)
    ctx.expect_overlap(focus_ring, barrel_body, axes="xy", min_overlap=0.105)
    ctx.expect_overlap(iris_ring, barrel_body, axes="xy", min_overlap=0.094)
    ctx.expect_gap(
        focus_ring,
        iris_ring,
        axis="z",
        min_gap=0.040,
        name="focus_ring_clear_of_iris_ring",
    )
    ctx.expect_gap(
        barrel_body,
        focus_ring,
        axis="z",
        min_gap=0.001,
        positive_elem=front_housing_shell,
        negative_elem=gear_tooth,
        name="front_housing_stays_forward_of_focus_teeth",
    )
    ctx.expect_gap(
        iris_ring,
        barrel_body,
        axis="z",
        min_gap=0.003,
        positive_elem=iris_grip,
        negative_elem=rear_mount_shell,
        name="iris_ring_sits_forward_of_rear_mount",
    )
    ctx.expect_gap(
        barrel_body,
        barrel_body,
        axis="z",
        min_gap=0.001,
        positive_elem=front_rim_shell,
        negative_elem=front_glass,
        name="front_glass_is_recessed_behind_front_rim",
    )
    ctx.expect_within(
        barrel_body,
        barrel_body,
        axes="xy",
        inner_elem=front_glass,
        outer_elem=front_rim_shell,
        name="front_glass_stays_inside_front_rim",
    )

    focus_rest = ctx.part_world_position(focus_ring)
    iris_rest = ctx.part_world_position(iris_ring)

    if focus_limits is not None and focus_limits.lower is not None and focus_limits.upper is not None:
        with ctx.pose({focus_rotation: focus_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_rotation_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="focus_rotation_lower_no_floating")
            ctx.expect_contact(
                focus_ring,
                barrel_body,
                elem_a=focus_pad,
                elem_b=focus_seat_shell,
                contact_tol=0.0015,
                name="focus_lower_pose_keeps_bearing_contact",
            )
        with ctx.pose({focus_rotation: focus_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_rotation_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="focus_rotation_upper_no_floating")
            ctx.expect_contact(
                focus_ring,
                barrel_body,
                elem_a=focus_pad,
                elem_b=focus_seat_shell,
                contact_tol=0.0015,
                name="focus_upper_pose_keeps_bearing_contact",
            )

    if iris_limits is not None and iris_limits.lower is not None and iris_limits.upper is not None:
        with ctx.pose({iris_rotation: iris_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="iris_rotation_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="iris_rotation_lower_no_floating")
            ctx.expect_contact(
                iris_ring,
                barrel_body,
                elem_a=iris_pad,
                elem_b=iris_seat_shell,
                contact_tol=0.0015,
                name="iris_lower_pose_keeps_bearing_contact",
            )
        with ctx.pose({iris_rotation: iris_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="iris_rotation_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="iris_rotation_upper_no_floating")
            ctx.expect_contact(
                iris_ring,
                barrel_body,
                elem_a=iris_pad,
                elem_b=iris_seat_shell,
                contact_tol=0.0015,
                name="iris_upper_pose_keeps_bearing_contact",
            )

    with ctx.pose({focus_rotation: 1.7, iris_rotation: -0.45}):
        ctx.expect_contact(
            focus_ring,
            barrel_body,
            elem_a=focus_pad,
            elem_b=focus_seat_shell,
            contact_tol=0.0015,
            name="focus_contact_persists_in_combined_pose",
        )
        ctx.expect_contact(
            iris_ring,
            barrel_body,
            elem_a=iris_pad,
            elem_b=iris_seat_shell,
            contact_tol=0.0015,
            name="iris_contact_persists_in_combined_pose",
        )
        ctx.expect_gap(
            focus_ring,
            iris_ring,
            axis="z",
            min_gap=0.040,
            name="rings_clear_each_other_in_combined_pose",
        )
        focus_rotated = ctx.part_world_position(focus_ring)
        iris_rotated = ctx.part_world_position(iris_ring)
        ctx.check(
            "focus_ring_rotates_without_barrel_extension",
            _position_delta(focus_rest, focus_rotated) <= 1e-6,
            f"Focus position moved from {focus_rest!r} to {focus_rotated!r}",
        )
        ctx.check(
            "iris_ring_rotates_without_axial_shift",
            _position_delta(iris_rest, iris_rotated) <= 1e-6,
            f"Iris position moved from {iris_rest!r} to {iris_rotated!r}",
        )

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
