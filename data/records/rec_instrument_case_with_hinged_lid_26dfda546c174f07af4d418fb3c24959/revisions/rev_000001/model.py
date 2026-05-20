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
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clarinet_case")

    shell_lower = model.material("shell_lower", rgba=(0.14, 0.14, 0.16, 1.0))
    shell_lid = model.material("shell_lid", rgba=(0.18, 0.18, 0.20, 1.0))
    liner = model.material("liner", rgba=(0.10, 0.10, 0.12, 1.0))
    hardware = model.material("hardware", rgba=(0.66, 0.67, 0.70, 1.0))
    latch_finish = model.material("latch_finish", rgba=(0.22, 0.22, 0.24, 1.0))

    case_length = 0.44
    case_width = 0.15
    lower_height = 0.052
    hinge_axis_y = -0.079
    hinge_axis_z = 0.060

    def rounded_section(
        length: float,
        width: float,
        radius: float,
        z: float,
        center_y: float,
    ) -> list[tuple[float, float, float]]:
        return [(x, y + center_y, z) for x, y in rounded_rect_profile(length, width, radius)]

    lower_body = model.part("lower_body")

    lower_floor = Box((0.420, 0.130, 0.004))
    lower_body.visual(
        lower_floor,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=shell_lower,
        name="base_floor",
    )

    lower_wall_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(case_length, case_width, 0.022),
        [rounded_rect_profile(0.420, 0.130, 0.016)],
        0.048,
        center=False,
    )
    lower_body.visual(
        mesh_from_geometry(lower_wall_geom, "lower_body_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=shell_lower,
        name="body_ring",
    )
    lower_body.visual(
        Box((0.392, 0.102, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=liner,
        name="liner_floor",
    )
    lower_body.visual(
        Box((0.135, 0.034, 0.016)),
        origin=Origin(xyz=(-0.105, 0.0, 0.012)),
        material=liner,
        name="joint_saddle",
    )
    lower_body.visual(
        Box((0.100, 0.040, 0.018)),
        origin=Origin(xyz=(0.035, 0.0, 0.013)),
        material=liner,
        name="center_saddle",
    )
    lower_body.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.150, 0.0, 0.014,)),
        material=liner,
        name="bell_pocket",
    )

    lower_body.visual(
        Cylinder(radius=0.006, length=0.14),
        origin=Origin(
            xyz=(-0.14, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware,
        name="left_hinge_barrel",
    )
    lower_body.visual(
        Cylinder(radius=0.006, length=0.14),
        origin=Origin(
            xyz=(0.14, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=hardware,
        name="right_hinge_barrel",
    )
    lower_body.visual(
        Box((0.132, 0.014, 0.006)),
        origin=Origin(xyz=(-0.14, -0.078, 0.055)),
        material=hardware,
        name="left_hinge_leaf",
    )
    lower_body.visual(
        Box((0.132, 0.014, 0.006)),
        origin=Origin(xyz=(0.14, -0.078, 0.055)),
        material=hardware,
        name="right_hinge_leaf",
    )

    latch_x_positions = {"left": -0.115, "right": 0.115}
    for side, x_pos in latch_x_positions.items():
        lower_body.visual(
            Box((0.004, 0.008, 0.014)),
            origin=Origin(xyz=(x_pos - 0.012, 0.079, 0.044)),
            material=hardware,
            name=f"{side}_latch_left_ear",
        )
        lower_body.visual(
            Box((0.004, 0.008, 0.014)),
            origin=Origin(xyz=(x_pos + 0.012, 0.079, 0.044)),
            material=hardware,
            name=f"{side}_latch_right_ear",
        )
        lower_body.visual(
            Box((0.030, 0.005, 0.010)),
            origin=Origin(xyz=(x_pos, 0.075, 0.042)),
            material=shell_lower,
            name=f"{side}_latch_mount",
        )

    lower_body.inertial = Inertial.from_geometry(
        Box((0.44, 0.15, 0.06)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    lid = model.part("lid")
    lid_center_y = 0.072

    lid_skirt_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(0.434, 0.132, 0.019),
        [rounded_rect_profile(0.418, 0.116, 0.014)],
        0.020,
        center=True,
    )
    lid.visual(
        mesh_from_geometry(lid_skirt_geom, "lid_skirt"),
        origin=Origin(xyz=(0.0, lid_center_y, 0.002)),
        material=shell_lid,
        name="lid_skirt",
    )

    lid_crown_geom = section_loft(
        [
            rounded_section(0.432, 0.130, 0.018, 0.010, lid_center_y),
            rounded_section(0.426, 0.126, 0.018, 0.018, lid_center_y),
            rounded_section(0.412, 0.118, 0.016, 0.028, lid_center_y),
            rounded_section(0.386, 0.094, 0.013, 0.039, lid_center_y),
        ]
    )
    lid.visual(
        mesh_from_geometry(lid_crown_geom, "domed_lid_crown"),
        material=shell_lid,
        name="domed_crown",
    )
    lid.visual(
        Box((0.390, 0.098, 0.003)),
        origin=Origin(xyz=(0.0, lid_center_y, 0.0095)),
        material=liner,
        name="lid_liner",
    )
    lid.visual(
        Cylinder(radius=0.006, length=0.14),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.304, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.141, -0.002)),
        material=hardware,
        name="catch_rail",
    )
    lid.visual(
        Box((0.336, 0.006, 0.014)),
        origin=Origin(xyz=(0.0, 0.138, 0.001)),
        material=shell_lid,
        name="front_rim",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.44, 0.14, 0.045)),
        mass=0.8,
        origin=Origin(xyz=(0.0, lid_center_y, 0.016)),
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(120.0),
        ),
    )

    def add_latch(name: str, x_pos: float) -> None:
        latch = model.part(name)
        latch.visual(
            Cylinder(radius=0.0035, length=0.020),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="pivot_barrel",
        )
        latch.visual(
            Box((0.022, 0.016, 0.014)),
            origin=Origin(xyz=(0.0, -0.004, 0.014)),
            material=latch_finish,
            name="clasp_spine",
        )
        latch.visual(
            Box((0.022, 0.007, 0.006)),
            origin=Origin(xyz=(0.0, -0.0145, 0.012)),
            material=latch_finish,
            name="hook",
        )
        latch.visual(
            Box((0.022, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, 0.006, 0.004)),
            material=latch_finish,
            name="finger_tab",
        )
        latch.inertial = Inertial.from_geometry(
            Box((0.022, 0.024, 0.022)),
            mass=0.08,
            origin=Origin(xyz=(0.0, -0.003, 0.011)),
        )

        model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent=lower_body,
            child=latch,
            origin=Origin(xyz=(x_pos, 0.082, 0.045)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=3.0,
                lower=-math.radians(70.0),
                upper=math.radians(15.0),
            ),
        )

    add_latch("left_latch", latch_x_positions["left"])
    add_latch("right_latch", latch_x_positions["right"])

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    lid = object_model.get_part("lid")
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")

    rear_hinge = object_model.get_articulation("rear_hinge")
    left_latch_hinge = object_model.get_articulation("left_latch_hinge")
    right_latch_hinge = object_model.get_articulation("right_latch_hinge")

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

    rear_limits = rear_hinge.motion_limits
    ctx.check(
        "rear_hinge_axis",
        tuple(rear_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected rear hinge axis along +x, got {rear_hinge.axis}",
    )
    ctx.check(
        "rear_hinge_limits",
        rear_limits is not None
        and rear_limits.lower == 0.0
        and rear_limits.upper is not None
        and rear_limits.upper >= math.radians(115.0),
        f"expected wide-opening lid limits, got {rear_limits}",
    )
    ctx.check(
        "front_latch_axes",
        tuple(left_latch_hinge.axis) == (1.0, 0.0, 0.0)
        and tuple(right_latch_hinge.axis) == (1.0, 0.0, 0.0),
        f"latch axes were {left_latch_hinge.axis} and {right_latch_hinge.axis}",
    )

    ctx.expect_contact(lid, lower_body, elem_a="hinge_barrel", elem_b="left_hinge_barrel")
    ctx.expect_contact(lid, lower_body, elem_a="hinge_barrel", elem_b="right_hinge_barrel")
    ctx.expect_contact(left_latch, lower_body, elem_a="pivot_barrel", elem_b="left_latch_left_ear")
    ctx.expect_contact(right_latch, lower_body, elem_a="pivot_barrel", elem_b="right_latch_left_ear")
    ctx.expect_contact(left_latch, lid, elem_a="hook", elem_b="catch_rail")
    ctx.expect_contact(right_latch, lid, elem_a="hook", elem_b="catch_rail")
    ctx.expect_overlap(lid, lower_body, axes="x", min_overlap=0.35)
    ctx.expect_overlap(lid, lower_body, axes="y", min_overlap=0.10)

    with ctx.pose({left_latch_hinge: -math.radians(60.0), right_latch_hinge: -math.radians(60.0)}):
        ctx.expect_contact(left_latch, lower_body, elem_a="pivot_barrel", elem_b="left_latch_left_ear")
        ctx.expect_contact(right_latch, lower_body, elem_a="pivot_barrel", elem_b="right_latch_left_ear")
        ctx.expect_gap(left_latch, lid, axis="y", min_gap=0.01, positive_elem="hook", negative_elem="catch_rail")
        ctx.expect_gap(right_latch, lid, axis="y", min_gap=0.01, positive_elem="hook", negative_elem="catch_rail")

    with ctx.pose(
        {
            rear_hinge: math.radians(110.0),
            left_latch_hinge: -math.radians(60.0),
            right_latch_hinge: -math.radians(60.0),
        }
    ):
        ctx.expect_contact(lid, lower_body, elem_a="hinge_barrel", elem_b="left_hinge_barrel")
        ctx.expect_contact(lid, lower_body, elem_a="hinge_barrel", elem_b="right_hinge_barrel")
        ctx.expect_gap(lid, lower_body, axis="z", min_gap=0.12, positive_elem="catch_rail")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
