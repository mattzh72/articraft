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
    DomeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fresnel_tripod_floor_lamp")

    matte_black = model.material("matte_black", rgba=(0.13, 0.13, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.18, 0.18, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.49, 0.52, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    fresnel_glass = model.material("fresnel_glass", rgba=(0.82, 0.85, 0.88, 0.65))

    def add_tripod_leg(part_name: str) -> None:
        leg = model.part(part_name)
        leg.visual(
            Box((0.060, 0.030, 0.024)),
            origin=Origin(xyz=(0.030, 0.0, -0.012)),
            material=matte_black,
            name="upper_brace",
        )
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.032, 0.0, -0.014),
                        (0.160, 0.0, -0.180),
                        (0.565, 0.0, -1.020),
                    ],
                    radius=0.016,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"{part_name}_tube",
            ),
            material=satin_black,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=(0.565, 0.0, -1.020)),
            material=rubber,
            name="foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.640, 0.080, 1.060)),
            mass=1.2,
            origin=Origin(xyz=(0.290, 0.0, -0.520)),
        )

    apex = model.part("apex_assembly")
    apex.visual(
        Cylinder(radius=0.110, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.010)),
        material=matte_black,
        name="apex_ring",
    )
    apex.visual(
        Cylinder(radius=0.050, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=matte_black,
        name="lower_collar",
    )
    apex.visual(
        Cylinder(radius=0.036, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.075)),
        material=satin_black,
        name="pivot_riser",
    )

    hinge_radius = 0.140
    leg_specs = (
        ("front_leg", 0.0),
        ("left_leg", 2.0 * math.pi / 3.0),
        ("right_leg", 4.0 * math.pi / 3.0),
    )
    apex.visual(
        Box((0.070, 0.050, 0.024)),
        origin=Origin(xyz=(hinge_radius, 0.0, 1.052)),
        material=matte_black,
        name="hinge_pad_front",
    )
    apex.visual(
        Box((0.070, 0.050, 0.024)),
        origin=Origin(
            xyz=(hinge_radius * math.cos(2.0 * math.pi / 3.0), hinge_radius * math.sin(2.0 * math.pi / 3.0), 1.052),
            rpy=(0.0, 0.0, 2.0 * math.pi / 3.0),
        ),
        material=matte_black,
        name="hinge_pad_left",
    )
    apex.visual(
        Box((0.070, 0.050, 0.024)),
        origin=Origin(
            xyz=(hinge_radius * math.cos(4.0 * math.pi / 3.0), hinge_radius * math.sin(4.0 * math.pi / 3.0), 1.052),
            rpy=(0.0, 0.0, 4.0 * math.pi / 3.0),
        ),
        material=matte_black,
        name="hinge_pad_right",
    )

    left_brace_mesh = tube_from_spline_points(
        [
            (0.015, -0.022, 1.060),
            (0.060, -0.120, 1.105),
            (0.140, -0.210, 1.200),
        ],
        radius=0.015,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    apex.visual(
        Box((0.140, 0.080, 0.130)),
        origin=Origin(xyz=(0.095, -0.180, 1.120)),
        material=matte_black,
        name="left_yoke_base",
    )
    apex.visual(
        mesh_from_geometry(left_brace_mesh, "left_yoke_brace"),
        material=satin_black,
        name="left_yoke_brace",
    )
    apex.visual(
        Box((0.130, 0.060, 0.190)),
        origin=Origin(xyz=(0.160, -0.220, 1.220)),
        material=matte_black,
        name="left_yoke_arm",
    )
    apex.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(
            xyz=(0.160, -0.210, 1.220),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_pivot_bushing",
    )

    right_brace_mesh = tube_from_spline_points(
        [
            (0.015, 0.022, 1.060),
            (0.060, 0.120, 1.105),
            (0.140, 0.210, 1.200),
        ],
        radius=0.015,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    apex.visual(
        Box((0.140, 0.080, 0.130)),
        origin=Origin(xyz=(0.095, 0.180, 1.120)),
        material=matte_black,
        name="right_yoke_base",
    )
    apex.visual(
        mesh_from_geometry(right_brace_mesh, "right_yoke_brace"),
        material=satin_black,
        name="right_yoke_brace",
    )
    apex.visual(
        Box((0.130, 0.060, 0.190)),
        origin=Origin(xyz=(0.160, 0.220, 1.220)),
        material=matte_black,
        name="right_yoke_arm",
    )
    apex.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(
            xyz=(0.160, 0.210, 1.220),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_pivot_bushing",
    )

    apex.inertial = Inertial.from_geometry(
        Box((0.420, 0.420, 1.280)),
        mass=4.2,
        origin=Origin(xyz=(0.040, 0.0, 0.980)),
    )

    add_tripod_leg("front_leg")
    add_tripod_leg("left_leg")
    add_tripod_leg("right_leg")

    head = model.part("lamp_head")
    head.visual(
        Cylinder(radius=0.165, length=0.180),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="main_shell",
    )
    head.visual(
        Cylinder(radius=0.185, length=0.050),
        origin=Origin(xyz=(0.165, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.152, length=0.008),
        origin=Origin(xyz=(0.172, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fresnel_glass,
        name="lens",
    )
    head.visual(
        mesh_from_geometry(
            DomeGeometry(
                0.145,
                radial_segments=32,
                height_segments=16,
                closed=True,
            )
            .rotate_y(-math.pi / 2.0)
            .translate(-0.040, 0.0, 0.0),
            "rear_dome",
        ),
        material=matte_black,
        name="rear_dome",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(
            xyz=(0.0, -0.170, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="left_trunnion",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(
            xyz=(0.0, 0.170, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="right_trunnion",
    )
    head.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.015, 0.0, 0.155),
                    (0.045, 0.0, 0.225),
                    (0.150, 0.0, 0.225),
                    (0.185, 0.0, 0.158),
                ],
                radius=0.008,
                samples_per_segment=16,
                radial_segments=14,
                cap_ends=True,
            ),
            "lamp_head_handle",
        ),
        material=steel,
        name="top_handle",
    )
    head.visual(
        Box((0.020, 0.028, 0.026)),
        origin=Origin(xyz=(0.020, 0.0, 0.150)),
        material=steel,
        name="rear_handle_mount",
    )
    head.visual(
        Box((0.020, 0.028, 0.026)),
        origin=Origin(xyz=(0.180, 0.0, 0.152)),
        material=steel,
        name="front_handle_mount",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.460, 0.380, 0.420)),
        mass=5.5,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    for part_name, angle in leg_specs:
        leg_part = model.get_part(part_name)
        model.articulation(
            f"apex_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=apex,
            child=leg_part,
            origin=Origin(
                xyz=(hinge_radius * math.cos(angle), hinge_radius * math.sin(angle), 1.040),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=0.0,
                upper=2.15,
            ),
        )

    model.articulation(
        "apex_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=apex,
        child=head,
        origin=Origin(xyz=(0.220, 0.0, 1.220)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=-0.65,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    apex = object_model.get_part("apex_assembly")
    head = object_model.get_part("lamp_head")
    front_leg = object_model.get_part("front_leg")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")

    front_hinge = object_model.get_articulation("apex_to_front_leg")
    left_hinge = object_model.get_articulation("apex_to_left_leg")
    right_hinge = object_model.get_articulation("apex_to_right_leg")
    head_tilt = object_model.get_articulation("apex_to_lamp_head")

    ctx.check(
        "all prompt-critical parts exist",
        all(part is not None for part in (apex, head, front_leg, left_leg, right_leg)),
    )

    ctx.expect_gap(
        apex,
        front_leg,
        axis="z",
        positive_elem="hinge_pad_front",
        negative_elem="upper_brace",
        max_gap=0.001,
        max_penetration=0.0,
        name="front leg seats under the apex hinge pad",
    )
    ctx.expect_gap(
        apex,
        left_leg,
        axis="z",
        positive_elem="hinge_pad_left",
        negative_elem="upper_brace",
        max_gap=0.001,
        max_penetration=0.0,
        name="left leg seats under the apex hinge pad",
    )
    ctx.expect_gap(
        apex,
        right_leg,
        axis="z",
        positive_elem="hinge_pad_right",
        negative_elem="upper_brace",
        max_gap=0.001,
        max_penetration=0.0,
        name="right leg seats under the apex hinge pad",
    )

    ctx.expect_contact(
        head,
        apex,
        elem_a="left_trunnion",
        elem_b="left_yoke_arm",
        contact_tol=0.001,
        name="left trunnion bears on the left yoke arm",
    )
    ctx.expect_contact(
        head,
        apex,
        elem_a="right_trunnion",
        elem_b="right_yoke_arm",
        contact_tol=0.001,
        name="right trunnion bears on the right yoke arm",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    rest_foot = _aabb_center(ctx.part_element_world_aabb(front_leg, elem="foot"))
    with ctx.pose({front_hinge: front_hinge.motion_limits.upper}):
        folded_foot = _aabb_center(ctx.part_element_world_aabb(front_leg, elem="foot"))

    front_hinge_xyz = front_hinge.origin.xyz
    ctx.check(
        "front tripod leg folds upward about its hinge",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[2] > rest_foot[2] + 0.35
        and front_hinge_xyz is not None
        and folded_foot[0] < front_hinge_xyz[0] - 0.20,
        details=(
            f"hinge={front_hinge_xyz}, rest_foot={rest_foot}, folded_foot={folded_foot}"
        ),
    )

    rest_bezel = _aabb_center(ctx.part_element_world_aabb(head, elem="front_bezel"))
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        tilted_bezel = _aabb_center(ctx.part_element_world_aabb(head, elem="front_bezel"))
        ctx.expect_contact(
            head,
            apex,
            elem_a="left_trunnion",
            elem_b="left_yoke_arm",
            contact_tol=0.001,
            name="left trunnion stays captured while the head tilts",
        )
        ctx.expect_contact(
            head,
            apex,
            elem_a="right_trunnion",
            elem_b="right_yoke_arm",
            contact_tol=0.001,
            name="right trunnion stays captured while the head tilts",
        )

    ctx.check(
        "lamp head tilts upward on the yoke",
        rest_bezel is not None
        and tilted_bezel is not None
        and tilted_bezel[2] > rest_bezel[2] + 0.08,
        details=f"rest_bezel={rest_bezel}, tilted_bezel={tilted_bezel}",
    )

    ctx.check(
        "leg hinges and head tilt expose realistic motion limits",
        front_hinge.motion_limits is not None
        and left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and head_tilt.motion_limits is not None
        and front_hinge.motion_limits.upper >= 0.9
        and head_tilt.motion_limits.lower <= -0.5
        and head_tilt.motion_limits.upper >= 0.7,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
