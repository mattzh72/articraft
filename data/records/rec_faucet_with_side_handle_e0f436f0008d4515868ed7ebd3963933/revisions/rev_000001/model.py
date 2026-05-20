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
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _visual_center_from_aabb(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _xy_radius(point: tuple[float, float, float] | None, pivot_xy: tuple[float, float]) -> float | None:
    if point is None:
        return None
    return math.hypot(point[0] - pivot_xy[0], point[1] - pivot_xy[1])


def _axis_pitch(axis: tuple[float, float, float]) -> float:
    return math.atan2(axis[0], axis[2])


def _normalize(axis: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in axis))
    return tuple(component / length for component in axis)


def _point_along_axis(axis: tuple[float, float, float], distance: float) -> tuple[float, float, float]:
    return (axis[0] * distance, axis[1] * distance, axis[2] * distance)


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gooseneck_kitchen_faucet")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_nozzle = model.material("dark_nozzle", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.056, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=stainless,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.038, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=stainless,
        name="base_column",
    )
    swivel_shell = LatheGeometry.from_shell_profiles(
        [
            (0.026, -0.018),
            (0.0245, -0.010),
            (0.0235, 0.010),
            (0.0245, 0.022),
        ],
        [
            (0.0172, -0.018),
            (0.0172, 0.020),
        ],
        segments=48,
    )
    body.visual(
        _mesh("faucet_swivel_shell", swivel_shell),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=brushed_steel,
        name="swivel_shell",
    )

    body.visual(
        Cylinder(radius=0.009, length=0.036),
        origin=Origin(xyz=(0.018, -0.026, 0.079), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="lever_bridge",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.020),
        origin=Origin(xyz=(0.046, -0.026, 0.080)),
        material=brushed_steel,
        name="lever_pedestal",
    )
    body.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(xyz=(0.046, -0.026, 0.082)),
        material=stainless,
        name="lever_shoulder",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.130, 0.110, 0.108)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.0152, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=brushed_steel,
        name="swivel_column",
    )

    spout_path = [
        (0.0, 0.0, -0.016),
        (0.0, 0.0, 0.080),
        (0.024, 0.0, 0.235),
        (0.120, 0.0, 0.382),
        (0.205, 0.0, 0.325),
        (0.209, 0.0, 0.305),
    ]
    spout.visual(
        _mesh(
            "gooseneck_spout_tube",
            tube_from_spline_points(
                spout_path,
                radius=0.0146,
                samples_per_segment=20,
                radial_segments=24,
                cap_ends=True,
            ),
        ),
        material=stainless,
        name="spout_tube",
    )

    nozzle_axis = _normalize((0.36, 0.0, -0.933))
    nozzle_pitch = _axis_pitch(nozzle_axis)
    nozzle_center = (0.232, 0.0, 0.244)
    nozzle_transition = LatheGeometry.from_shell_profiles(
        [
            (0.0152, -0.075),
            (0.0162, -0.030),
            (0.0168, 0.000),
            (0.0171, 0.018),
        ],
        [
            (0.0122, -0.070),
            (0.0135, -0.030),
            (0.0147, 0.000),
            (0.0158, 0.018),
        ],
        segments=40,
    )
    spout.visual(
        _mesh("faucet_nozzle_transition", nozzle_transition),
        origin=Origin(xyz=nozzle_center, rpy=(0.0, nozzle_pitch, 0.0)),
        material=brushed_steel,
        name="nozzle_transition",
    )
    nozzle_seat = ExtrudeWithHolesGeometry(
        _circle_profile(0.0176, segments=48),
        [_circle_profile(0.0158, segments=48)],
        0.010,
        center=True,
    )
    spout.visual(
        _mesh("faucet_nozzle_seat", nozzle_seat),
        origin=Origin(
            xyz=tuple(
                nozzle_center[i] + _point_along_axis(nozzle_axis, 0.023)[i] for i in range(3)
            ),
            rpy=(0.0, nozzle_pitch, 0.0),
        ),
        material=brushed_steel,
        name="nozzle_seat",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.260, 0.060, 0.420)),
        mass=1.5,
        origin=Origin(xyz=(0.110, 0.0, 0.185)),
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0155, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=stainless,
        name="lever_hub",
    )
    lever.visual(
        _mesh(
            "side_lever_arm",
            sweep_profile_along_spline(
                [
                    (0.000, 0.000, 0.013),
                    (0.014, 0.001, 0.014),
                    (0.036, 0.010, 0.015),
                    (0.062, 0.024, 0.013),
                ],
                profile=rounded_rect_profile(0.012, 0.0048, radius=0.0016),
                samples_per_segment=18,
                cap_profile=True,
            ),
        ),
        material=stainless,
        name="lever_arm",
    )
    lever.visual(
        Box((0.034, 0.016, 0.005)),
        origin=Origin(xyz=(0.076, 0.032, 0.013), rpy=(0.0, 0.0, 0.42)),
        material=brushed_steel,
        name="lever_paddle",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.112, 0.060, 0.028)),
        mass=0.16,
        origin=Origin(xyz=(0.052, 0.018, 0.013)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.0106, length=0.120),
        origin=Origin(
            xyz=_point_along_axis(nozzle_axis, 0.000),
            rpy=(0.0, nozzle_pitch, 0.0),
        ),
        material=rubber,
        name="wand_stem",
    )
    wand.visual(
        Cylinder(radius=0.0150, length=0.072),
        origin=Origin(
            xyz=_point_along_axis(nozzle_axis, 0.086),
            rpy=(0.0, nozzle_pitch, 0.0),
        ),
        material=stainless,
        name="wand_body",
    )
    wand.visual(
        Cylinder(radius=0.0128, length=0.026),
        origin=Origin(
            xyz=_point_along_axis(nozzle_axis, 0.043),
            rpy=(0.0, nozzle_pitch, 0.0),
        ),
        material=stainless,
        name="wand_neck",
    )
    wand.visual(
        _mesh(
            "faucet_wand_collar",
            ExtrudeWithHolesGeometry(
                _circle_profile(0.0172, segments=48),
                [_circle_profile(0.0110, segments=48)],
                0.002,
                center=True,
            ),
        ),
        origin=Origin(
            xyz=_point_along_axis(nozzle_axis, 0.029),
            rpy=(0.0, nozzle_pitch, 0.0),
        ),
        material=brushed_steel,
        name="wand_collar",
    )
    wand.visual(
        Cylinder(radius=0.0164, length=0.030),
        origin=Origin(
            xyz=_point_along_axis(nozzle_axis, 0.129),
            rpy=(0.0, nozzle_pitch, 0.0),
        ),
        material=dark_nozzle,
        name="spray_face",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.110, 0.040, 0.260)),
        mass=0.28,
        origin=Origin(xyz=_point_along_axis(nozzle_axis, 0.010)),
    )

    model.articulation(
        "body_to_spout",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.046, -0.026, 0.090)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=-0.95,
            upper=0.95,
        ),
    )
    model.articulation(
        "spout_to_wand",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=wand,
        origin=Origin(xyz=nozzle_center),
        axis=nozzle_axis,
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.30,
            lower=0.0,
            upper=0.085,
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

    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    wand = object_model.get_part("wand")

    spout_joint = object_model.get_articulation("body_to_spout")
    lever_joint = object_model.get_articulation("body_to_lever")
    wand_joint = object_model.get_articulation("spout_to_wand")

    ctx.expect_contact(
        lever,
        body,
        elem_a="lever_hub",
        elem_b="lever_pedestal",
        contact_tol=0.0015,
        name="lever hub is mounted on the side pedestal",
    )
    ctx.expect_overlap(
        spout,
        body,
        axes="xy",
        elem_a="swivel_column",
        elem_b="swivel_shell",
        min_overlap=0.030,
        name="spout swivel column stays centered in the body bearing",
    )
    ctx.expect_contact(
        spout,
        body,
        elem_a="swivel_column",
        elem_b="base_column",
        contact_tol=1e-6,
        name="spout bearing column is seated on the faucet body",
    )
    ctx.expect_within(
        wand,
        spout,
        axes="y",
        inner_elem="wand_body",
        outer_elem="nozzle_seat",
        margin=0.004,
        name="wand stays centered laterally in the nozzle seat",
    )
    ctx.expect_contact(
        wand,
        spout,
        elem_a="wand_collar",
        elem_b="nozzle_seat",
        contact_tol=0.0005,
        name="wand collar seats against the nozzle shell",
    )

    lever_pivot = (0.046, -0.026)
    lever_tip_centers = []
    for q in (-0.60, 0.60):
        with ctx.pose({lever_joint: q}):
            center = _visual_center_from_aabb(
                ctx.part_element_world_aabb(lever, elem="lever_paddle")
            )
            lever_tip_centers.append(center)
    lever_left, lever_right = lever_tip_centers
    lever_r0 = _xy_radius(lever_left, lever_pivot)
    lever_r1 = _xy_radius(lever_right, lever_pivot)
    ctx.check(
        "lever rotates about the valve axis",
        lever_left is not None
        and lever_right is not None
        and lever_r0 is not None
        and lever_r1 is not None
        and abs(lever_r0 - lever_r1) < 0.008
        and abs(lever_left[2] - lever_right[2]) < 0.010
        and abs(lever_left[1] - lever_right[1]) > 0.025,
        details=f"left={lever_left}, right={lever_right}, radii={(lever_r0, lever_r1)}",
    )

    nozzle_centers = []
    for q in (0.0, math.pi / 2.0):
        with ctx.pose({spout_joint: q}):
            center = _visual_center_from_aabb(
                ctx.part_element_world_aabb(spout, elem="nozzle_seat")
            )
            nozzle_centers.append(center)
    nozzle_front, nozzle_side = nozzle_centers
    spout_r0 = _xy_radius(nozzle_front, (0.0, 0.0))
    spout_r1 = _xy_radius(nozzle_side, (0.0, 0.0))
    ctx.check(
        "gooseneck spout swivels about the vertical mounting axis",
        nozzle_front is not None
        and nozzle_side is not None
        and spout_r0 is not None
        and spout_r1 is not None
        and abs(spout_r0 - spout_r1) < 0.012
        and abs(nozzle_front[2] - nozzle_side[2]) < 0.006
        and abs(nozzle_front[1] - nozzle_side[1]) > 0.120,
        details=f"front={nozzle_front}, side={nozzle_side}, radii={(spout_r0, spout_r1)}",
    )

    with ctx.pose({wand_joint: 0.0}):
        rest_head = _visual_center_from_aabb(ctx.part_element_world_aabb(wand, elem="spray_face"))
    with ctx.pose({wand_joint: 0.085}):
        extended_head = _visual_center_from_aabb(
            ctx.part_element_world_aabb(wand, elem="spray_face")
        )
    ctx.check(
        "spray wand extends outward from the spout end",
        rest_head is not None
        and extended_head is not None
        and extended_head[0] > rest_head[0] + 0.025
        and extended_head[2] < rest_head[2] - 0.070,
        details=f"rest={rest_head}, extended={extended_head}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
