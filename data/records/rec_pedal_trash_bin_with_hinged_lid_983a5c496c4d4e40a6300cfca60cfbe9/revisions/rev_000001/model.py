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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wrap_profile_onto_surface,
)


BODY_RADIUS = 0.150
BODY_HEIGHT = 0.596
LID_RADIUS = 0.147
HINGE_AXIS_Y = -0.147
HINGE_AXIS_Z = 0.599
PEDAL_AXIS_Y = 0.149
PEDAL_AXIS_Z = 0.052


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_body_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.000),
            (0.132, 0.000),
            (0.145, 0.010),
            (0.150, 0.050),
            (0.150, 0.585),
            (0.147, 0.592),
            (0.140, 0.596),
        ],
        [
            (0.000, 0.014),
            (0.124, 0.014),
            (0.141, 0.026),
            (0.141, 0.580),
            (0.134, 0.590),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def _build_lid_shell_mesh():
    return LatheGeometry.from_shell_profiles(
        [
            (0.000, 0.072),
            (0.028, 0.070),
            (0.080, 0.058),
            (0.122, 0.028),
            (0.142, 0.010),
            (0.147, 0.004),
        ],
        [
            (0.000, 0.060),
            (0.024, 0.058),
            (0.074, 0.048),
            (0.116, 0.022),
            (0.136, 0.008),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_step_bin")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.66, 0.69, 0.72, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.44, 0.46, 0.49, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        _mesh("step_bin_body_shell", _build_body_shell_mesh()),
        material=stainless,
        name="body_shell",
    )
    body.visual(
        _mesh(
            "rear_hinge_band",
            wrap_profile_onto_surface(
                rounded_rect_profile(0.118, 0.074, radius=0.010, corner_segments=8),
                body,
                thickness=0.0024,
                point_hint=(0.0, -BODY_RADIUS, BODY_HEIGHT - 0.052),
                visible_relief=0.0008,
            ),
        ),
        material=brushed_steel,
        name="rear_hinge_band",
    )
    body.visual(
        Box((0.036, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, -0.147, 0.5845)),
        material=hinge_dark,
        name="rear_hinge_saddle",
    )
    body.visual(
        _mesh(
            "front_pedal_backing",
            wrap_profile_onto_surface(
                rounded_rect_profile(0.118, 0.046, radius=0.008, corner_segments=8),
                body,
                thickness=0.0020,
                point_hint=(0.0, BODY_RADIUS, PEDAL_AXIS_Z),
                visible_relief=0.0008,
            ),
        ),
        material=brushed_steel,
        name="front_pedal_backing",
    )
    body.visual(
        Box((0.012, 0.018, 0.022)),
        origin=Origin(xyz=(-0.056, PEDAL_AXIS_Y, PEDAL_AXIS_Z)),
        material=hinge_dark,
        name="pedal_bracket_left",
    )
    body.visual(
        Box((0.012, 0.018, 0.022)),
        origin=Origin(xyz=(0.056, PEDAL_AXIS_Y, PEDAL_AXIS_Z)),
        material=hinge_dark,
        name="pedal_bracket_right",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.155, length=0.610),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
    )

    lid = model.part("lid")
    lid.visual(
        _mesh("step_bin_lid_shell", _build_lid_shell_mesh()),
        origin=Origin(xyz=(0.0, LID_RADIUS, -0.004)),
        material=stainless,
        name="lid_shell",
    )
    lid.visual(
        Cylinder(radius=0.0055, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="lid_hinge_sleeve",
    )
    lid.visual(
        Box((0.030, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, 0.012, 0.010)),
        material=hinge_dark,
        name="lid_hinge_bridge",
    )
    lid.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.080),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.074, 0.030)),
    )

    pedal = model.part("pedal")
    pedal.visual(
        Cylinder(radius=0.0035, length=0.108),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_dark,
        name="pedal_pivot_shaft",
    )
    pedal.visual(
        Box((0.012, 0.026, 0.016)),
        origin=Origin(xyz=(-0.048, 0.012, 0.003)),
        material=pedal_black,
        name="pedal_arm_left",
    )
    pedal.visual(
        Box((0.012, 0.026, 0.016)),
        origin=Origin(xyz=(0.048, 0.012, 0.003)),
        material=pedal_black,
        name="pedal_arm_right",
    )
    pedal.visual(
        Cylinder(radius=0.0052, length=0.160),
        origin=Origin(
            xyz=(0.0, 0.024, 0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pedal_black,
        name="pedal_bar",
    )
    pedal.visual(
        Box((0.148, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.024, 0.006)),
        material=pedal_black,
        name="pedal_tread",
    )
    pedal.inertial = Inertial.from_geometry(
        Box((0.160, 0.040, 0.030)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.018, 0.004)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "body_to_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, PEDAL_AXIS_Y, PEDAL_AXIS_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    lid_hinge = object_model.get_articulation("body_to_lid")
    pedal_hinge = object_model.get_articulation("body_to_pedal")

    lid_shell = lid.get_visual("lid_shell")
    pedal_bar = pedal.get_visual("pedal_bar")

    ctx.check(
        "step bin parts exist",
        body is not None and lid is not None and pedal is not None,
        details="body, lid, and pedal should all be present",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_shell,
            negative_elem="body_shell",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid sits just above the rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_shell,
            elem_b="body_shell",
            min_overlap=0.260,
            name="closed lid covers the cylindrical opening",
        )

    ctx.expect_origin_gap(
        pedal,
        body,
        axis="y",
        min_gap=0.145,
        max_gap=0.153,
        name="pedal pivot sits on the front wall",
    )
    ctx.expect_origin_gap(
        pedal,
        body,
        axis="z",
        min_gap=0.045,
        max_gap=0.060,
        name="pedal pivot sits low near the base",
    )

    closed_lid_aabb = None
    open_lid_aabb = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lid_aabb = ctx.part_element_world_aabb(lid, elem=lid_shell)
    with ctx.pose({lid_hinge: 1.10}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem=lid_shell)

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.040,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_pedal_aabb = None
    pressed_pedal_aabb = None
    with ctx.pose({pedal_hinge: 0.0}):
        closed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem=pedal_bar)
    with ctx.pose({pedal_hinge: 0.36}):
        pressed_pedal_aabb = ctx.part_element_world_aabb(pedal, elem=pedal_bar)

    ctx.check(
        "pedal bar rotates downward when pressed",
        closed_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[0][2] < closed_pedal_aabb[0][2] - 0.008,
        details=f"closed={closed_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )
    ctx.check(
        "pedal bar swings outward slightly when pressed",
        closed_pedal_aabb is not None
        and pressed_pedal_aabb is not None
        and pressed_pedal_aabb[1][1] > closed_pedal_aabb[1][1] + 0.001,
        details=f"closed={closed_pedal_aabb}, pressed={pressed_pedal_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
