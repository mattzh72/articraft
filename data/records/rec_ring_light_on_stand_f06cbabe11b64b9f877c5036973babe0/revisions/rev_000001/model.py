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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
    rotate_to_x: bool = False,
) -> object:
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        height=thickness,
        center=True,
    )
    if rotate_to_x:
        geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_ring_light")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.25, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.78, 1.0))
    diffuser = model.material("diffuser", rgba=(0.95, 0.95, 0.93, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=matte_black,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.027, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=matte_black,
        name="leg_collar",
    )
    base.visual(
        _annulus_mesh(
            outer_radius=0.018,
            inner_radius=0.014,
            thickness=0.480,
            name="outer_sleeve_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=charcoal,
        name="outer_sleeve",
    )
    base.visual(
        _annulus_mesh(
            outer_radius=0.025,
            inner_radius=0.0145,
            thickness=0.050,
            name="top_clamp_body_mesh",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=matte_black,
        name="top_clamp_body",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(
            xyz=(0.027, 0.0, 0.555),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=aluminum,
        name="clamp_stem",
    )
    base.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.048, 0.0, 0.555)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_geom = tube_from_spline_points(
            [
                (0.025 * c, 0.025 * s, 0.115),
                (0.165 * c, 0.165 * s, 0.070),
                (0.305 * c, 0.305 * s, 0.020),
            ],
            radius=0.009,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_geom, f"tripod_leg_mesh_{index}"),
            material=matte_black,
            name=f"tripod_leg_{index}",
        )
        base.visual(
            Sphere(radius=0.011),
            origin=Origin(xyz=(0.305 * c, 0.305 * s, 0.020)),
            material=rubber,
            name=f"tripod_foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.66, 0.66, 0.64)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
    )

    upper_mast = model.part("upper_mast")
    upper_mast.visual(
        Cylinder(radius=0.0115, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=aluminum,
        name="inner_tube",
    )
    upper_mast.visual(
        Cylinder(radius=0.018, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        material=matte_black,
        name="head_receiver",
    )
    upper_mast.visual(
        Box((0.040, 0.072, 0.110)),
        origin=Origin(xyz=(0.008, 0.0, 0.470)),
        material=matte_black,
        name="tilt_knuckle",
    )
    upper_mast.visual(
        Box((0.020, 0.018, 0.180)),
        origin=Origin(xyz=(0.029, 0.233, 0.510)),
        material=matte_black,
        name="right_yoke_arm",
    )
    upper_mast.visual(
        Box((0.020, 0.018, 0.180)),
        origin=Origin(xyz=(0.029, -0.233, 0.510)),
        material=matte_black,
        name="left_yoke_arm",
    )
    upper_mast.visual(
        Box((0.020, 0.472, 0.040)),
        origin=Origin(xyz=(0.015, 0.0, 0.410)),
        material=matte_black,
        name="lower_yoke_bridge",
    )
    upper_mast.visual(
        Box((0.030, 0.060, 0.090)),
        origin=Origin(xyz=(0.004, 0.0, 0.372)),
        material=matte_black,
        name="support_spine",
    )
    upper_mast.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(
            xyz=(0.029, 0.233, 0.580),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=aluminum,
        name="right_yoke_cap",
    )
    upper_mast.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(
            xyz=(0.029, -0.233, 0.580),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=aluminum,
        name="left_yoke_cap",
    )
    upper_mast.inertial = Inertial.from_geometry(
        Box((0.48, 0.52, 0.98)),
        mass=1.4,
        origin=Origin(xyz=(-0.010, 0.0, 0.220)),
    )

    model.articulation(
        "mast_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.590)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=0.18,
            lower=0.0,
            upper=0.320,
        ),
    )

    head = model.part("head")
    head.visual(
        _annulus_mesh(
            outer_radius=0.215,
            inner_radius=0.128,
            thickness=0.032,
            name="ring_housing_mesh",
            rotate_to_x=True,
        ),
        material=charcoal,
        name="ring_housing",
    )
    head.visual(
        _annulus_mesh(
            outer_radius=0.198,
            inner_radius=0.138,
            thickness=0.003,
            name="ring_diffuser_mesh",
            rotate_to_x=True,
        ),
        origin=Origin(xyz=(0.0175, 0.0, 0.0)),
        material=diffuser,
        name="front_diffuser",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.214, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_black,
        name="right_trunnion",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.214, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=matte_black,
        name="left_trunnion",
    )
    head.visual(
        Box((0.028, 0.018, 0.024)),
        origin=Origin(xyz=(-0.006, 0.095, 0.204)),
        material=matte_black,
        name="right_handle_pivot_housing",
    )
    head.visual(
        Box((0.028, 0.018, 0.024)),
        origin=Origin(xyz=(-0.006, -0.095, 0.204)),
        material=matte_black,
        name="left_handle_pivot_housing",
    )
    head.visual(
        Box((0.044, 0.072, 0.032)),
        origin=Origin(xyz=(0.018, 0.0, -0.188)),
        material=matte_black,
        name="control_receiver",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.46, 0.46, 0.08)),
        mass=1.9,
        origin=Origin(),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_mast,
        child=head,
        origin=Origin(xyz=(0.055, 0.0, 0.580)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.2,
            lower=-0.75,
            upper=0.95,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(0.014, 0.095, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=aluminum,
        name="right_handle_pivot",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.016),
        origin=Origin(
            xyz=(0.014, -0.095, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=aluminum,
        name="left_handle_pivot",
    )
    handle.visual(
        Box((0.014, 0.018, 0.012)),
        origin=Origin(xyz=(0.016, 0.095, 0.008)),
        material=aluminum,
        name="right_handle_root",
    )
    handle.visual(
        Box((0.014, 0.018, 0.012)),
        origin=Origin(xyz=(0.016, -0.095, 0.008)),
        material=aluminum,
        name="left_handle_root",
    )
    right_handle_rail = tube_from_spline_points(
        [
            (0.022, 0.095, 0.008),
            (0.024, 0.095, 0.034),
            (0.024, 0.088, 0.058),
            (0.024, 0.080, 0.080),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    left_handle_rail = tube_from_spline_points(
        [
            (0.022, -0.095, 0.008),
            (0.024, -0.095, 0.034),
            (0.024, -0.088, 0.058),
            (0.024, -0.080, 0.080),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    handle.visual(
        mesh_from_geometry(right_handle_rail, "right_handle_rail_mesh"),
        material=aluminum,
        name="right_handle_rail",
    )
    handle.visual(
        mesh_from_geometry(left_handle_rail, "left_handle_rail_mesh"),
        material=aluminum,
        name="left_handle_rail",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.166),
        origin=Origin(
            xyz=(0.024, 0.0, 0.080),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=aluminum,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.040, 0.210, 0.130)),
        mass=0.18,
        origin=Origin(xyz=(0.020, 0.0, 0.045)),
    )

    model.articulation(
        "handle_fold",
        ArticulationType.REVOLUTE,
        parent=head,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
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

    base = object_model.get_part("base")
    upper_mast = object_model.get_part("upper_mast")
    head = object_model.get_part("head")
    handle = object_model.get_part("handle")

    mast_extension = object_model.get_articulation("mast_extension")
    head_tilt = object_model.get_articulation("head_tilt")
    handle_fold = object_model.get_articulation("handle_fold")

    mast_upper = mast_extension.motion_limits.upper if mast_extension.motion_limits else 0.0
    tilt_upper = head_tilt.motion_limits.upper if head_tilt.motion_limits else 0.0
    handle_upper = handle_fold.motion_limits.upper if handle_fold.motion_limits else 0.0

    with ctx.pose({mast_extension: 0.0, head_tilt: 0.0, handle_fold: 0.0}):
        ctx.expect_within(
            upper_mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="inner mast stays centered in outer sleeve at rest",
        )
        ctx.expect_overlap(
            upper_mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.34,
            name="collapsed mast retains substantial insertion",
        )
        ctx.expect_overlap(
            handle,
            head,
            axes="y",
            elem_a="handle_grip",
            elem_b="ring_housing",
            min_overlap=0.12,
            name="folded handle spans across the ring width",
        )
        ctx.expect_gap(
            handle,
            head,
            axis="z",
            positive_elem="handle_grip",
            negative_elem="ring_housing",
            min_gap=0.0,
            max_gap=0.090,
            name="folded handle stays just above the top of the ring housing",
        )
        ctx.expect_gap(
            handle,
            head,
            axis="x",
            positive_elem="handle_grip",
            negative_elem="ring_housing",
            min_gap=0.0,
            max_gap=0.020,
            name="folded handle stays close to the front of the ring housing",
        )
        right_yoke_contact = ctx.expect_contact(
            head,
            upper_mast,
            elem_a="right_trunnion",
            elem_b="right_yoke_cap",
            contact_tol=0.0015,
            name="right trunnion contacts the yoke cap",
        )
        left_yoke_contact = ctx.expect_contact(
            head,
            upper_mast,
            elem_a="left_trunnion",
            elem_b="left_yoke_cap",
            contact_tol=0.0015,
            name="left trunnion contacts the yoke cap",
        )
        right_handle_contact = ctx.expect_contact(
            handle,
            head,
            elem_a="right_handle_pivot",
            elem_b="right_handle_pivot_housing",
            contact_tol=0.0015,
            name="right handle pivot seats in the housing",
        )
        left_handle_contact = ctx.expect_contact(
            handle,
            head,
            elem_a="left_handle_pivot",
            elem_b="left_handle_pivot_housing",
            contact_tol=0.0015,
            name="left handle pivot seats in the housing",
        )
        ctx.check(
            "head and handle are mounted through their pivot hardware",
            right_yoke_contact and left_yoke_contact and right_handle_contact and left_handle_contact,
            details="Both tilt trunnions and both handle pivots should seat against their mount hardware.",
        )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({mast_extension: mast_upper, head_tilt: 0.0, handle_fold: 0.0}):
        ctx.expect_within(
            upper_mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_sleeve",
            margin=0.003,
            name="extended mast stays centered in outer sleeve",
        )
        ctx.expect_overlap(
            upper_mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.075,
            name="extended mast keeps retained insertion",
        )
        extended_head_pos = ctx.part_world_position(head)
    ctx.check(
        "mast extension raises the head",
        rest_head_pos is not None
        and extended_head_pos is not None
        and extended_head_pos[2] > rest_head_pos[2] + 0.25,
        details=f"rest={rest_head_pos}, extended={extended_head_pos}",
    )

    receiver_rest_aabb = ctx.part_element_world_aabb(head, elem="control_receiver")
    with ctx.pose({mast_extension: 0.0, head_tilt: tilt_upper * 0.75, handle_fold: 0.0}):
        receiver_tilted_aabb = ctx.part_element_world_aabb(head, elem="control_receiver")
    ctx.check(
        "positive tilt raises the lower front receiver module",
        receiver_rest_aabb is not None
        and receiver_tilted_aabb is not None
        and receiver_tilted_aabb[0][2] > receiver_rest_aabb[0][2] + 0.03,
        details=f"rest={receiver_rest_aabb}, tilted={receiver_tilted_aabb}",
    )

    grip_folded_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({mast_extension: 0.0, head_tilt: 0.0, handle_fold: handle_upper}):
        grip_open_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")
    ctx.check(
        "handle opens outward from the ring",
        grip_folded_aabb is not None
        and grip_open_aabb is not None
        and grip_open_aabb[1][0] > grip_folded_aabb[1][0] + 0.05,
        details=f"folded={grip_folded_aabb}, open={grip_open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
