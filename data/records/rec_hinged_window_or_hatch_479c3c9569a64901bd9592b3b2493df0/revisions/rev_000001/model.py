from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gullwing_service_hatch")

    body_paint = model.material("body_paint", rgba=(0.76, 0.78, 0.80, 1.0))
    inner_liner = model.material("inner_liner", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.52, 0.55, 0.58, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.34, 0.36, 0.39, 1.0))

    panel_width = 1.74
    panel_height = 1.14
    panel_radius = 0.12
    panel_thickness = 0.036
    opening_width = 1.08
    opening_height = 0.64
    opening_radius = 0.08
    hinge_axis_y = 0.032
    hinge_axis_z = opening_height * 0.5

    outer_panel_profile = rounded_rect_profile(
        panel_width, panel_height, panel_radius, corner_segments=10
    )
    opening_profile = rounded_rect_profile(
        opening_width, opening_height, opening_radius, corner_segments=10
    )

    body_side = model.part("body_side")
    body_side.inertial = Inertial.from_geometry(
        Box((panel_width, 0.18, panel_height + 0.12)),
        mass=55.0,
        origin=Origin(xyz=(0.0, -0.05, 0.0)),
    )

    body_side.visual(
        _mesh(
            "body_side_outer_ring",
            ExtrudeWithHolesGeometry(
                outer_panel_profile,
                [opening_profile],
                height=panel_thickness,
                center=True,
            ).rotate_x(-pi / 2.0),
        ),
        material=body_paint,
        name="panel_ring",
    )

    body_side.visual(
        Box((opening_width + 0.34, 0.050, 0.10)),
        origin=Origin(xyz=(0.0, -0.008, hinge_axis_z + 0.13)),
        material=body_paint,
        name="upper_brow",
    )
    body_side.visual(
        Box((panel_width * 0.94, 0.070, 0.12)),
        origin=Origin(xyz=(0.0, -0.014, -0.44)),
        material=body_paint,
        name="lower_sill",
    )

    body_side.visual(
        _mesh(
            "body_side_opening_jamb",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(
                    opening_width + 0.18,
                    opening_height + 0.16,
                    opening_radius + 0.03,
                    corner_segments=10,
                ),
                [
                    rounded_rect_profile(
                        opening_width + 0.02,
                        opening_height + 0.02,
                        opening_radius + 0.01,
                        corner_segments=10,
                    )
                ],
                height=0.100,
                center=True,
            ).rotate_x(-pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, -0.068, 0.0)),
        material=inner_liner,
        name="opening_jamb",
    )

    body_side.visual(
        Box((opening_width + 0.12, 0.044, 0.024)),
        origin=Origin(xyz=(0.0, hinge_axis_y - 0.014, hinge_axis_z + 0.028)),
        material=body_paint,
        name="hinge_header",
    )

    for x_pos, length, suffix in (
        (-0.36, 0.20, "left"),
        (0.36, 0.20, "right"),
    ):
        body_side.visual(
            Cylinder(radius=0.016, length=length),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name=f"body_hinge_knuckle_{suffix}",
        )

    striker_z = -opening_height * 0.5 + 0.064
    for x_pos, suffix in ((-0.29, "left"), (0.29, "right")):
        body_side.visual(
            Box((0.084, 0.072, 0.120)),
            origin=Origin(xyz=(x_pos, -0.080, striker_z - 0.050)),
            material=inner_liner,
            name=f"striker_pedestal_{suffix}",
        )
        body_side.visual(
            Box((0.050, 0.050, 0.058)),
            origin=Origin(xyz=(x_pos, -0.042, striker_z + 0.004)),
            material=latch_metal,
            name=f"striker_mount_{suffix}",
        )
        body_side.visual(
            Cylinder(radius=0.010, length=0.072),
            origin=Origin(
                xyz=(x_pos, -0.018, striker_z + 0.024),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name=f"striker_bar_{suffix}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((opening_width - 0.04, 0.070, opening_height)),
        mass=16.0,
        origin=Origin(xyz=(0.0, -0.030, -opening_height * 0.5)),
    )

    lid_outer_width = opening_width - 0.034
    lid_outer_height = opening_height - 0.030
    lid_center_y = -0.030
    lid_center_z = -(lid_outer_height * 0.5 + 0.012)

    lid.visual(
        _mesh(
            "service_hatch_outer_skin",
            ExtrudeGeometry(
                rounded_rect_profile(
                    lid_outer_width,
                    lid_outer_height,
                    opening_radius - 0.010,
                    corner_segments=10,
                ),
                height=0.028,
                center=True,
            ).rotate_x(-pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, lid_center_y, lid_center_z)),
        material=body_paint,
        name="outer_skin",
    )

    lid.visual(
        _mesh(
            "service_hatch_inner_frame",
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(
                    lid_outer_width - 0.08,
                    lid_outer_height - 0.08,
                    opening_radius - 0.018,
                    corner_segments=10,
                ),
                [
                    rounded_rect_profile(
                        lid_outer_width - 0.34,
                        lid_outer_height - 0.26,
                        max(0.035, opening_radius - 0.045),
                        corner_segments=8,
                    )
                ],
                height=0.024,
                center=True,
            ).rotate_x(-pi / 2.0),
        ),
        origin=Origin(xyz=(0.0, -0.022, lid_center_z + 0.004)),
        material=dark_trim,
        name="inner_frame",
    )

    lid.visual(
        Box((lid_outer_width * 0.70, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, -0.010, -0.050)),
        material=hinge_metal,
        name="hinge_leaf",
    )
    for x_pos, suffix in ((-0.18, "left"), (0.00, "center"), (0.18, "right")):
        lid.visual(
            Box((0.060, 0.018, 0.050)),
            origin=Origin(xyz=(x_pos, -0.010, -0.025)),
            material=hinge_metal,
            name=f"hinge_ear_{suffix}",
        )
    lid.visual(
        Box((0.080, 0.024, lid_outer_height * 0.70)),
        origin=Origin(xyz=(-0.26, -0.020, lid_center_z + 0.02)),
        material=dark_trim,
        name="left_rib",
    )
    lid.visual(
        Box((0.080, 0.024, lid_outer_height * 0.70)),
        origin=Origin(xyz=(0.26, -0.020, lid_center_z + 0.02)),
        material=dark_trim,
        name="right_rib",
    )
    lid.visual(
        Box((lid_outer_width * 0.64, 0.022, 0.054)),
        origin=Origin(xyz=(0.0, -0.022, lid_center_z - 0.23)),
        material=dark_trim,
        name="bottom_beam",
    )

    latch_pivot_z = lid_center_z - 0.185
    for x_pos, suffix in ((-0.29, "left"), (0.29, "right")):
        lid.visual(
            Box((0.060, 0.016, 0.050)),
            origin=Origin(xyz=(x_pos, -0.018, latch_pivot_z - 0.010)),
            material=dark_trim,
            name=f"latch_mount_{suffix}",
        )

    for x_pos, length, suffix in (
        (-0.18, 0.14, "left"),
        (0.00, 0.18, "center"),
        (0.18, 0.14, "right"),
    ):
        lid.visual(
            Cylinder(radius=0.014, length=length),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"lid_hinge_knuckle_{suffix}",
        )

    left_latch_dog = model.part("left_latch_dog")
    left_latch_dog.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="pivot_barrel",
    )
    left_latch_dog.visual(
        Box((0.020, 0.008, 0.064)),
        origin=Origin(xyz=(0.0, -0.004, -0.032)),
        material=latch_metal,
        name="dog_plate",
    )
    left_latch_dog.visual(
        Box((0.034, 0.008, 0.014)),
        origin=Origin(xyz=(0.010, -0.004, -0.066)),
        material=latch_metal,
        name="dog_toe",
    )
    left_latch_dog.inertial = Inertial.from_geometry(
        Box((0.034, 0.016, 0.078)),
        mass=0.35,
        origin=Origin(xyz=(0.006, 0.0, -0.034)),
    )

    right_latch_dog = model.part("right_latch_dog")
    right_latch_dog.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="pivot_barrel",
    )
    right_latch_dog.visual(
        Box((0.020, 0.008, 0.064)),
        origin=Origin(xyz=(0.0, -0.004, -0.032)),
        material=latch_metal,
        name="dog_plate",
    )
    right_latch_dog.visual(
        Box((0.034, 0.008, 0.014)),
        origin=Origin(xyz=(-0.010, -0.004, -0.066)),
        material=latch_metal,
        name="dog_toe",
    )
    right_latch_dog.inertial = Inertial.from_geometry(
        Box((0.034, 0.016, 0.078)),
        mass=0.35,
        origin=Origin(xyz=(-0.006, 0.0, -0.034)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body_side,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=0.0,
            upper=1.30,
        ),
    )
    model.articulation(
        "left_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=left_latch_dog,
        origin=Origin(xyz=(-0.29, -0.002, latch_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-1.10,
            upper=0.35,
        ),
    )
    model.articulation(
        "right_latch_pivot",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=right_latch_dog,
        origin=Origin(xyz=(0.29, -0.002, latch_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-0.35,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body_side = object_model.get_part("body_side")
    lid = object_model.get_part("lid")
    lid_hinge = object_model.get_articulation("lid_hinge")
    left_latch_dog = object_model.get_part("left_latch_dog")
    right_latch_dog = object_model.get_part("right_latch_dog")
    left_latch_pivot = object_model.get_articulation("left_latch_pivot")
    right_latch_pivot = object_model.get_articulation("right_latch_pivot")

    ctx.check(
        "lid hinge axis runs longitudinally",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "latch dogs rotate on panel-normal pivots",
        tuple(left_latch_pivot.axis) == (0.0, 1.0, 0.0)
        and tuple(right_latch_pivot.axis) == (0.0, 1.0, 0.0),
        details=(
            f"left_axis={left_latch_pivot.axis}, "
            f"right_axis={right_latch_pivot.axis}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0, left_latch_pivot: 0.0, right_latch_pivot: 0.0}):
        ctx.expect_overlap(
            lid,
            body_side,
            axes="xz",
            min_overlap=0.60,
            name="closed lid occupies the body-side hatch zone",
        )
        ctx.expect_overlap(
            left_latch_dog,
            lid,
            axes="xz",
            min_overlap=0.03,
            name="left latch dog stays mounted along the lid lower edge",
        )
        ctx.expect_overlap(
            right_latch_dog,
            lid,
            axes="xz",
            min_overlap=0.03,
            name="right latch dog stays mounted along the lid lower edge",
        )
        ctx.expect_contact(
            left_latch_dog,
            lid,
            elem_a="dog_plate",
            elem_b="latch_mount_left",
            contact_tol=1e-6,
            name="left latch dog is physically seated on its lid-side latch mount",
        )
        ctx.expect_contact(
            right_latch_dog,
            lid,
            elem_a="dog_plate",
            elem_b="latch_mount_right",
            contact_tol=1e-6,
            name="right latch dog is physically seated on its lid-side latch mount",
        )

    rest_skin_aabb = ctx.part_element_world_aabb(lid, elem="outer_skin")
    with ctx.pose({lid_hinge: 1.15}):
        open_skin_aabb = ctx.part_element_world_aabb(lid, elem="outer_skin")

    ctx.check(
        "lid swings outward and upward",
        rest_skin_aabb is not None
        and open_skin_aabb is not None
        and open_skin_aabb[1][1] > rest_skin_aabb[1][1] + 0.45
        and open_skin_aabb[0][2] > rest_skin_aabb[0][2] + 0.08,
        details=f"rest={rest_skin_aabb}, open={open_skin_aabb}",
    )

    left_toe_rest = ctx.part_element_world_aabb(left_latch_dog, elem="dog_toe")
    with ctx.pose({left_latch_pivot: -0.85}):
        left_toe_rotated = ctx.part_element_world_aabb(left_latch_dog, elem="dog_toe")
    right_toe_rest = ctx.part_element_world_aabb(right_latch_dog, elem="dog_toe")
    with ctx.pose({right_latch_pivot: 0.85}):
        right_toe_rotated = ctx.part_element_world_aabb(right_latch_dog, elem="dog_toe")

    ctx.check(
        "left latch dog sweeps through a rotary arc",
        left_toe_rest is not None
        and left_toe_rotated is not None
        and abs(
            ((left_toe_rotated[0][0] + left_toe_rotated[1][0]) * 0.5)
            - ((left_toe_rest[0][0] + left_toe_rest[1][0]) * 0.5)
        )
        > 0.02,
        details=f"rest={left_toe_rest}, rotated={left_toe_rotated}",
    )
    ctx.check(
        "right latch dog sweeps through a rotary arc",
        right_toe_rest is not None
        and right_toe_rotated is not None
        and abs(
            ((right_toe_rotated[0][0] + right_toe_rotated[1][0]) * 0.5)
            - ((right_toe_rest[0][0] + right_toe_rest[1][0]) * 0.5)
        )
        > 0.02,
        details=f"rest={right_toe_rest}, rotated={right_toe_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
