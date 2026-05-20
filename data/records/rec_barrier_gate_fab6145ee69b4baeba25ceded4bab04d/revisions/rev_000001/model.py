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
    rounded_rect_profile,
)


def _circle_profile(radius: float, *, segments: int = 20) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_bollard_barrier")

    base_coat = model.material("base_coat", rgba=(0.22, 0.24, 0.26, 1.0))
    galvanized_steel = model.material("galvanized_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    receiver_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.044, 0.046, 0.006, corner_segments=6),
        [_circle_profile(0.007, segments=20)],
        0.010,
        center=True,
        cap=True,
        closed=True,
    )
    receiver_geom.rotate_x(math.pi / 2.0)
    receiver_mesh = mesh_from_geometry(receiver_geom, "padlock_receiver_tab")

    base_assembly = model.part("base_assembly")
    base_assembly.visual(
        Box((0.22, 0.18, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=base_coat,
        name="elem_plate",
    )
    base_assembly.visual(
        Box((0.024, 0.028, 0.045)),
        origin=Origin(xyz=(0.068, -0.031, 0.0365)),
        material=base_coat,
        name="elem_clevis_left",
    )
    base_assembly.visual(
        Box((0.024, 0.028, 0.045)),
        origin=Origin(xyz=(0.068, 0.031, 0.0365)),
        material=base_coat,
        name="elem_clevis_right",
    )
    base_assembly.visual(
        Box((0.026, 0.018, 0.042)),
        origin=Origin(xyz=(0.020, 0.037, 0.035)),
        material=hardware_dark,
        name="elem_latch_stop",
    )
    base_assembly.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.068, -0.045, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="elem_hinge_pin_head_left",
    )
    base_assembly.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.068, 0.045, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="elem_hinge_pin_head_right",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((0.075, 0.055), (-0.075, 0.055), (0.075, -0.055), (-0.075, -0.055))
    ):
        base_assembly.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x_pos, y_pos, 0.017)),
            material=hardware_dark,
            name=f"elem_anchor_{index}",
        )
    base_assembly.inertial = Inertial.from_geometry(
        Box((0.22, 0.18, 0.080)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    bollard_post = model.part("bollard_post")
    bollard_post.visual(
        Cylinder(radius=0.016, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_dark,
        name="elem_hinge_knuckle",
    )
    bollard_post.visual(
        Box((0.032, 0.030, 0.030)),
        origin=Origin(xyz=(-0.016, 0.0, 0.031)),
        material=hardware_dark,
        name="elem_hinge_connector",
    )
    bollard_post.visual(
        Box((0.056, 0.056, 0.042)),
        origin=Origin(xyz=(-0.050, 0.0, 0.043)),
        material=hardware_dark,
        name="elem_post_foot",
    )
    bollard_post.visual(
        Cylinder(radius=0.046, length=0.048),
        origin=Origin(xyz=(-0.050, 0.0, 0.088)),
        material=galvanized_steel,
        name="elem_base_collar",
    )
    bollard_post.visual(
        Cylinder(radius=0.038, length=0.800),
        origin=Origin(xyz=(-0.050, 0.0, 0.464)),
        material=galvanized_steel,
        name="elem_post_shaft",
    )
    bollard_post.visual(
        Sphere(radius=0.039),
        origin=Origin(xyz=(-0.050, 0.0, 0.864)),
        material=galvanized_steel,
        name="elem_post_cap",
    )
    bollard_post.visual(
        receiver_mesh,
        origin=Origin(xyz=(-0.048, 0.033, 0.048)),
        material=hardware_dark,
        name="elem_receiver_tab",
    )
    bollard_post.inertial = Inertial.from_geometry(
        Box((0.120, 0.090, 0.900)),
        mass=12.0,
        origin=Origin(xyz=(-0.050, 0.0, 0.450)),
    )

    model.articulation(
        "base_to_post_hinge",
        ArticulationType.REVOLUTE,
        parent=base_assembly,
        child=bollard_post,
        origin=Origin(xyz=(0.068, 0.0, 0.031)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(88.0),
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

    base = object_model.get_part("base_assembly")
    post = object_model.get_part("bollard_post")
    hinge = object_model.get_articulation("base_to_post_hinge")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            post,
            base,
            axis="z",
            positive_elem="elem_post_shaft",
            negative_elem="elem_plate",
            min_gap=0.07,
            name="upright shaft clears the base plate",
        )
        ctx.expect_gap(
            post,
            base,
            axis="z",
            positive_elem="elem_receiver_tab",
            negative_elem="elem_latch_stop",
            max_gap=0.003,
            max_penetration=1e-5,
            name="receiver tab seats on the latch stop in the upright pose",
        )
        ctx.expect_overlap(
            post,
            base,
            axes="xy",
            elem_a="elem_receiver_tab",
            elem_b="elem_latch_stop",
            min_overlap=0.009,
            name="receiver tab aligns over the latch stop",
        )
        closed_cap_center = _aabb_center(ctx.part_element_world_aabb(post, elem="elem_post_cap"))

    with ctx.pose({hinge: math.radians(88.0)}):
        open_cap_center = _aabb_center(ctx.part_element_world_aabb(post, elem="elem_post_cap"))

    ctx.check(
        "post folds forward toward a horizontal parked position",
        closed_cap_center is not None
        and open_cap_center is not None
        and open_cap_center[0] > closed_cap_center[0] + 0.75
        and open_cap_center[2] < closed_cap_center[2] - 0.70,
        details=f"closed_cap_center={closed_cap_center}, open_cap_center={open_cap_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
