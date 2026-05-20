from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _circle_profile(radius: float, *, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _tube_shell_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments=48),
            [_circle_profile(inner_radius, segments=48)],
            length,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _fluted_focus_ring_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
):
    outer_profile = [
        (
            (
                outer_radius
                - 0.0018
                * (
                    0.5
                    - 0.5 * math.cos(16.0 * (2.0 * math.pi * index) / 96.0)
                )
            )
            * math.cos((2.0 * math.pi * index) / 96.0),
            (
                outer_radius
                - 0.0018
                * (
                    0.5
                    - 0.5 * math.cos(16.0 * (2.0 * math.pi * index) / 96.0)
                )
            )
            * math.sin((2.0 * math.pi * index) / 96.0),
        )
        for index in range(96)
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_profile,
            [_circle_profile(inner_radius, segments=48)],
            length,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _yz_section(
    x_pos: float,
    *,
    center_y: float,
    width_y: float,
    height_z: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, center_y + y_val, z_val)
        for y_val, z_val in rounded_rect_profile(
            width_y,
            height_z,
            corner_radius,
            corner_segments=6,
        )
    ]


def _prism_housing_mesh(name: str, *, side: float):
    return mesh_from_geometry(
        section_loft(
            [
                _yz_section(
                    0.038,
                    center_y=side * 0.038,
                    width_y=0.028,
                    height_z=0.034,
                    corner_radius=0.009,
                ),
                _yz_section(
                    0.072,
                    center_y=side * 0.040,
                    width_y=0.050,
                    height_z=0.056,
                    corner_radius=0.014,
                ),
                _yz_section(
                    0.104,
                    center_y=side * 0.042,
                    width_y=0.060,
                    height_z=0.066,
                    corner_radius=0.017,
                ),
                _yz_section(
                    0.126,
                    center_y=side * 0.042,
                    width_y=0.052,
                    height_z=0.058,
                    corner_radius=0.015,
                ),
            ]
        ),
        name,
    )


def _bridge_frame_mesh(name: str, *, side: float):
    upper_rib = (
        BoxGeometry((0.082 if side < 0.0 else 0.076, 0.020, 0.012))
        .rotate_z(side * 0.24)
        .translate(0.050, side * 0.022, 0.034)
    )
    lower_rib = (
        BoxGeometry((0.082 if side < 0.0 else 0.076, 0.020, 0.012))
        .rotate_z(side * 0.24)
        .translate(0.050, side * 0.022, -0.034)
    )

    if side < 0.0:
        bridge_geom = BoxGeometry((0.018, 0.010, 0.074)).translate(0.009, side * 0.016, 0.000)
        bridge_geom.merge(
            BoxGeometry((0.020, 0.014, 0.018)).translate(0.010, side * 0.016, 0.019)
        )
        bridge_geom.merge(
            BoxGeometry((0.020, 0.014, 0.018)).translate(0.010, side * 0.016, -0.019)
        )
    else:
        bridge_geom = BoxGeometry((0.020, 0.020, 0.018)).translate(0.010, side * 0.010, 0.000)
        bridge_geom.merge(BoxGeometry((0.006, 0.008, 0.030)).translate(0.015, side * 0.008, 0.019))
        bridge_geom.merge(BoxGeometry((0.006, 0.008, 0.030)).translate(0.015, side * 0.008, -0.019))

    bridge_geom.merge(upper_rib)
    bridge_geom.merge(lower_rib)
    return mesh_from_geometry(bridge_geom, name)


def _populate_body_half(
    part,
    *,
    side: float,
    rubber_material,
    trim_material,
    hinge_material,
    lens_material,
    prism_mesh,
    bridge_frame_mesh,
    objective_shell_mesh,
    eyepiece_shell_mesh,
    eyecup_shell_mesh,
) -> None:
    part.visual(prism_mesh, material=rubber_material, name="prism_housing")
    part.visual(
        bridge_frame_mesh,
        origin=Origin(),
        material=rubber_material,
        name="bridge_frame",
    )

    part.visual(
        objective_shell_mesh,
        origin=Origin(
            xyz=(0.148, side * 0.046, 0.000),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=rubber_material,
        name="objective_tube_shell",
    )
    part.visual(
        objective_shell_mesh,
        origin=Origin(
            xyz=(0.176, side * 0.046, 0.000),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=trim_material,
        name="objective_front_bumper",
    )
    part.visual(
        Cylinder(radius=0.0296, length=0.0025),
        origin=Origin(
            xyz=(0.201, side * 0.046, 0.000),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=lens_material,
        name="objective_lens",
    )

    part.visual(
        eyepiece_shell_mesh,
        origin=Origin(
            xyz=(0.030, side * 0.038, 0.000),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=trim_material,
        name="eyepiece_shell",
    )
    part.visual(
        Box((0.012, 0.003, 0.0012)),
        origin=Origin(xyz=(0.030, side * 0.038, 0.0201)),
        material=hinge_material,
        name="eyepiece_reference_mark",
    )
    part.visual(
        eyecup_shell_mesh,
        origin=Origin(
            xyz=(0.010, side * 0.038, 0.000),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=trim_material,
        name="eyecup_shell",
    )
    part.visual(
        Cylinder(radius=0.0179, length=0.0025),
        origin=Origin(
            xyz=(0.002, side * 0.038, 0.000),
            rpy=(0.000, math.pi / 2.0, 0.000),
        ),
        material=lens_material,
        name="ocular_lens",
    )

    if side < 0.0:
        part.visual(
            Cylinder(radius=0.0115, length=0.016),
            origin=Origin(xyz=(0.000, 0.000, 0.019)),
            material=hinge_material,
            name="hinge_upper_knuckle",
        )
        part.visual(
            Cylinder(radius=0.0115, length=0.016),
            origin=Origin(xyz=(0.000, 0.000, -0.019)),
            material=hinge_material,
            name="hinge_lower_knuckle",
        )
    else:
        part.visual(
            Cylinder(radius=0.0108, length=0.022),
            origin=Origin(xyz=(0.000, 0.000, 0.000)),
            material=hinge_material,
            name="hinge_center_knuckle",
        )


def _make_focus_ring(part, *, material, ring_shell_mesh) -> None:
    part.visual(
        ring_shell_mesh,
        origin=Origin(rpy=(0.000, math.pi / 2.0, 0.000)),
        material=material,
        name="focus_ring_shell",
    )
    part.visual(
        Box((0.016, 0.004, 0.0022)),
        origin=Origin(xyz=(0.000, 0.000, 0.0239)),
        material=material,
        name="focus_ring_index_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="aviation_pilot_binoculars")

    rubber_black = model.material("rubber_black", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_black = model.material("trim_black", rgba=(0.18, 0.19, 0.20, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    lens_blue = model.material("lens_blue", rgba=(0.36, 0.53, 0.63, 0.42))

    objective_shell_mesh = _tube_shell_mesh(
        "binocular_objective_shell",
        outer_radius=0.035,
        inner_radius=0.0295,
        length=0.108,
    )
    eyepiece_shell_mesh = _tube_shell_mesh(
        "binocular_eyepiece_shell",
        outer_radius=0.0205,
        inner_radius=0.0176,
        length=0.044,
    )
    eyecup_shell_mesh = _tube_shell_mesh(
        "binocular_eyecup_shell",
        outer_radius=0.0245,
        inner_radius=0.0178,
        length=0.016,
    )
    focus_ring_shell_mesh = _fluted_focus_ring_mesh(
        "binocular_focus_ring_shell",
        inner_radius=0.0209,
        outer_radius=0.0248,
        length=0.022,
    )

    left_prism_mesh = _prism_housing_mesh("left_prism_housing_mesh", side=-1.0)
    right_prism_mesh = _prism_housing_mesh("right_prism_housing_mesh", side=1.0)
    left_bridge_frame_mesh = _bridge_frame_mesh("left_bridge_frame_mesh", side=-1.0)
    right_bridge_frame_mesh = _bridge_frame_mesh("right_bridge_frame_mesh", side=1.0)
    left_body = model.part("left_body")
    _populate_body_half(
        left_body,
        side=-1.0,
        rubber_material=rubber_black,
        trim_material=trim_black,
        hinge_material=hinge_gray,
        lens_material=lens_blue,
        prism_mesh=left_prism_mesh,
        bridge_frame_mesh=left_bridge_frame_mesh,
        objective_shell_mesh=objective_shell_mesh,
        eyepiece_shell_mesh=eyepiece_shell_mesh,
        eyecup_shell_mesh=eyecup_shell_mesh,
    )

    right_body = model.part("right_body")
    _populate_body_half(
        right_body,
        side=1.0,
        rubber_material=rubber_black,
        trim_material=trim_black,
        hinge_material=hinge_gray,
        lens_material=lens_blue,
        prism_mesh=right_prism_mesh,
        bridge_frame_mesh=right_bridge_frame_mesh,
        objective_shell_mesh=objective_shell_mesh,
        eyepiece_shell_mesh=eyepiece_shell_mesh,
        eyecup_shell_mesh=eyecup_shell_mesh,
    )

    left_focus_ring = model.part("left_focus_ring")
    _make_focus_ring(left_focus_ring, material=trim_black, ring_shell_mesh=focus_ring_shell_mesh)

    right_focus_ring = model.part("right_focus_ring")
    _make_focus_ring(right_focus_ring, material=trim_black, ring_shell_mesh=focus_ring_shell_mesh)

    model.articulation(
        "binocular_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=0.22,
        ),
    )
    model.articulation(
        "left_focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=left_body,
        child=left_focus_ring,
        origin=Origin(xyz=(0.030, -0.038, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "right_focus_rotation",
        ArticulationType.CONTINUOUS,
        parent=right_body,
        child=right_focus_ring,
        origin=Origin(xyz=(0.030, 0.038, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
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

    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    left_focus_ring = object_model.get_part("left_focus_ring")
    right_focus_ring = object_model.get_part("right_focus_ring")
    hinge = object_model.get_articulation("binocular_hinge")
    left_focus_joint = object_model.get_articulation("left_focus_rotation")
    right_focus_joint = object_model.get_articulation("right_focus_rotation")
    left_reference_mark = left_body.get_visual("eyepiece_reference_mark")
    right_reference_mark = right_body.get_visual("eyepiece_reference_mark")
    left_index_tab = left_focus_ring.get_visual("focus_ring_index_tab")
    right_index_tab = right_focus_ring.get_visual("focus_ring_index_tab")

    ctx.check(
        "binocular articulation layout is present",
        all(
            item is not None
            for item in (
                left_body,
                right_body,
                left_focus_ring,
                right_focus_ring,
                hinge,
                left_focus_joint,
                right_focus_joint,
                left_reference_mark,
                right_reference_mark,
                left_index_tab,
                right_index_tab,
            )
        ),
        details="Expected both barrel bodies, both focus rings, and all three articulations.",
    )

    ctx.check(
        "hinge and focus axes match the intended mechanisms",
        hinge.axis == (0.0, 0.0, 1.0)
        and left_focus_joint.axis == (1.0, 0.0, 0.0)
        and right_focus_joint.axis == (1.0, 0.0, 0.0),
        details=(
            f"hinge_axis={hinge.axis}, "
            f"left_focus_axis={left_focus_joint.axis}, "
            f"right_focus_axis={right_focus_joint.axis}"
        ),
    )

    ctx.check(
        "focus rings are continuous rotations",
        left_focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and right_focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and left_focus_joint.motion_limits is not None
        and right_focus_joint.motion_limits is not None
        and left_focus_joint.motion_limits.lower is None
        and left_focus_joint.motion_limits.upper is None
        and right_focus_joint.motion_limits.lower is None
        and right_focus_joint.motion_limits.upper is None,
        details=(
            f"left_limits={left_focus_joint.motion_limits}, "
            f"right_limits={right_focus_joint.motion_limits}"
        ),
    )

    ctx.expect_overlap(
        left_focus_ring,
        left_body,
        axes="yz",
        elem_a="focus_ring_shell",
        elem_b="eyepiece_shell",
        min_overlap=0.038,
        name="left focus ring stays centered around the left eyepiece",
    )
    ctx.expect_overlap(
        right_focus_ring,
        right_body,
        axes="yz",
        elem_a="focus_ring_shell",
        elem_b="eyepiece_shell",
        min_overlap=0.038,
        name="right focus ring stays centered around the right eyepiece",
    )
    ctx.expect_overlap(
        left_focus_ring,
        left_body,
        axes="x",
        elem_a="focus_ring_shell",
        elem_b="eyepiece_shell",
        min_overlap=0.020,
        name="left focus ring covers the eyepiece sleeve axially",
    )
    ctx.expect_overlap(
        right_focus_ring,
        right_body,
        axes="x",
        elem_a="focus_ring_shell",
        elem_b="eyepiece_shell",
        min_overlap=0.020,
        name="right focus ring covers the eyepiece sleeve axially",
    )

    ctx.expect_gap(
        right_body,
        left_body,
        axis="y",
        positive_elem="eyepiece_shell",
        negative_elem="eyepiece_shell",
        min_gap=0.028,
        name="eyepiece barrels stay separated at minimum interpupillary setting",
    )
    ctx.expect_gap(
        right_body,
        left_body,
        axis="y",
        positive_elem="objective_tube_shell",
        negative_elem="objective_tube_shell",
        min_gap=0.018,
        name="objective barrels stay separated at minimum interpupillary setting",
    )

    rest_right_focus_position = ctx.part_world_position(right_focus_ring)
    upper_limit = hinge.motion_limits.upper if hinge.motion_limits is not None else None
    with ctx.pose({hinge: upper_limit if upper_limit is not None else 0.22}):
        opened_right_focus_position = ctx.part_world_position(right_focus_ring)
        ctx.expect_gap(
            right_body,
            left_body,
            axis="y",
            positive_elem="objective_tube_shell",
            negative_elem="objective_tube_shell",
            min_gap=0.040,
            name="objective barrels spread farther apart when the hinge opens",
        )

    ctx.check(
        "hinge increases interpupillary distance",
        rest_right_focus_position is not None
        and opened_right_focus_position is not None
        and opened_right_focus_position[1] > rest_right_focus_position[1] + 0.004,
        details=(
            f"rest={rest_right_focus_position}, "
            f"opened={opened_right_focus_position}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
