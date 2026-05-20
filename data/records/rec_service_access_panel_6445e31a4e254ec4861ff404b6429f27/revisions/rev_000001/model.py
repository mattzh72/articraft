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
    ExtrudeGeometry,
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
    model = ArticulatedObject(name="premium_service_access_panel")

    painted_metal = model.material("painted_metal", rgba=(0.80, 0.82, 0.84, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.60, 0.62, 0.65, 1.0))
    polymer_dark = model.material("polymer_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    elastomer_dark = model.material("elastomer_dark", rgba=(0.08, 0.09, 0.10, 1.0))

    outer_w = 0.760
    outer_h = 0.560
    opening_w = 0.624
    opening_h = 0.444
    door_w = 0.620
    door_h = 0.440
    hinge_axis_x = -0.318
    hinge_axis_z = 0.008
    door_left_gap = 0.008
    door_center_from_hinge = door_left_gap + door_w / 2.0

    def _section(width: float, height: float, radius: float, z: float, *, x0: float) -> list[tuple[float, float, float]]:
        return [(x + x0, y, z) for x, y in rounded_rect_profile(width, height, radius, corner_segments=10)]

    bezel_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_w, outer_h, 0.038, corner_segments=10),
            [rounded_rect_profile(opening_w, opening_h, 0.022, corner_segments=10)],
            height=0.003,
            center=True,
        ),
        "service_panel_bezel",
    )
    door_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _section(door_w, door_h, 0.021, -0.001, x0=door_center_from_hinge),
                _section(door_w - 0.008, door_h - 0.006, 0.020, -0.007, x0=door_center_from_hinge),
                _section(door_w - 0.020, door_h - 0.016, 0.017, -0.016, x0=door_center_from_hinge),
                _section(door_w - 0.028, door_h - 0.024, 0.014, -0.024, x0=door_center_from_hinge),
            ]
        ),
        "service_panel_door_shell",
    )
    door_trim_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(door_w, door_h, 0.021, corner_segments=10),
            [rounded_rect_profile(0.514, 0.334, 0.012, corner_segments=10)],
            height=0.0025,
            center=True,
        ),
        "service_panel_trim_ring",
    )
    latch_bezel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.094, 0.040, 0.010, corner_segments=8),
            0.008,
            center=True,
        ),
        "service_panel_latch_bezel",
    )

    frame = model.part("frame")
    frame.visual(bezel_mesh, origin=Origin(xyz=(0.0, 0.0, 0.0015)), material=painted_metal, name="bezel")
    frame.visual(
        Box((0.003, opening_h, 0.032)),
        origin=Origin(xyz=(-opening_w / 2.0, 0.0, -0.016)),
        material=painted_metal,
        name="left_reveal",
    )
    frame.visual(
        Box((0.003, opening_h, 0.032)),
        origin=Origin(xyz=(opening_w / 2.0, 0.0, -0.016)),
        material=painted_metal,
        name="right_reveal",
    )
    frame.visual(
        Box((opening_w, 0.003, 0.032)),
        origin=Origin(xyz=(0.0, opening_h / 2.0, -0.016)),
        material=painted_metal,
        name="top_reveal",
    )
    frame.visual(
        Box((opening_w, 0.003, 0.032)),
        origin=Origin(xyz=(0.0, -opening_h / 2.0, -0.016)),
        material=painted_metal,
        name="bottom_reveal",
    )
    frame.visual(
        Box((0.016, 0.390, 0.004)),
        origin=Origin(xyz=(-0.308, 0.0, -0.010)),
        material=elastomer_dark,
        name="seal_strip",
    )
    frame.visual(
        Box((0.012, 0.110, 0.010)),
        origin=Origin(xyz=(-0.330, 0.145, 0.000)),
        material=satin_metal,
        name="frame_upper_leaf",
    )
    frame.visual(
        Box((0.012, 0.110, 0.010)),
        origin=Origin(xyz=(-0.330, -0.145, 0.000)),
        material=satin_metal,
        name="frame_lower_leaf",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(hinge_axis_x, 0.145, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="frame_upper_knuckle",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.110),
        origin=Origin(xyz=(hinge_axis_x, -0.145, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="frame_lower_knuckle",
    )
    frame.visual(
        Box((0.004, 0.070, 0.012)),
        origin=Origin(xyz=(0.3125, 0.0, -0.010)),
        material=satin_metal,
        name="keeper_plate",
    )
    frame.visual(
        Box((0.010, 0.085, 0.014)),
        origin=Origin(xyz=(0.318, 0.0, -0.010)),
        material=polymer_dark,
        name="keeper_body",
    )
    frame.inertial = Inertial.from_geometry(
        Box((outer_w, outer_h, 0.040)),
        mass=4.4,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )

    panel = model.part("service_panel")
    panel.visual(door_shell_mesh, material=painted_metal, name="panel_shell")
    panel.visual(
        door_trim_mesh,
        origin=Origin(xyz=(door_center_from_hinge, 0.0, 0.000)),
        material=painted_metal,
        name="door_trim_ring",
    )
    panel.visual(
        Box((0.012, 0.160, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.000)),
        material=satin_metal,
        name="door_hinge_leaf",
    )
    panel.visual(
        Cylinder(radius=0.006, length=0.140),
        origin=Origin(xyz=(0.000, 0.0, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="door_knuckle",
    )
    panel.visual(
        latch_bezel_mesh,
        origin=Origin(xyz=(0.550, 0.000, 0.002)),
        material=polymer_dark,
        name="latch_bezel",
    )
    panel.visual(
        Box((0.014, 0.028, 0.010)),
        origin=Origin(xyz=(0.550, 0.000, 0.006)),
        material=elastomer_dark,
        name="latch_grip",
    )
    panel.visual(
        Box((0.036, 0.005, 0.002)),
        origin=Origin(xyz=(0.550, 0.000, 0.011)),
        material=satin_metal,
        name="latch_indicator",
    )
    panel.visual(
        Box((0.006, 0.050, 0.010)),
        origin=Origin(xyz=(0.625, 0.000, -0.010)),
        material=satin_metal,
        name="latch_tongue",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.628, 0.440, 0.030)),
        mass=3.2,
        origin=Origin(xyz=(0.318, 0.0, -0.010)),
    )

    model.articulation(
        "frame_to_service_panel",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=panel,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=0.0,
            upper=2.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    panel = object_model.get_part("service_panel")
    hinge = object_model.get_articulation("frame_to_service_panel")
    bezel = frame.get_visual("bezel")
    left_reveal = frame.get_visual("left_reveal")
    right_reveal = frame.get_visual("right_reveal")
    top_reveal = frame.get_visual("top_reveal")
    keeper_plate = frame.get_visual("keeper_plate")
    panel_shell = panel.get_visual("panel_shell")
    latch_bezel = panel.get_visual("latch_bezel")
    latch_tongue = panel.get_visual("latch_tongue")

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
    ctx.warn_if_articulation_overlaps(max_pose_samples=24)

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            panel,
            frame,
            axis="x",
            positive_elem=panel_shell,
            negative_elem=left_reveal,
            min_gap=0.0002,
            max_gap=0.0020,
            name="left_perimeter_gap_is_tight",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="x",
            positive_elem=right_reveal,
            negative_elem=panel_shell,
            min_gap=0.0002,
            max_gap=0.0020,
            name="right_perimeter_gap_is_tight",
        )
        ctx.expect_gap(
            frame,
            panel,
            axis="y",
            positive_elem=top_reveal,
            negative_elem=panel_shell,
            min_gap=0.0002,
            max_gap=0.0020,
            name="top_perimeter_gap_is_tight",
        )
        ctx.expect_contact(
            frame,
            panel,
            elem_a=keeper_plate,
            elem_b=latch_tongue,
            contact_tol=0.0006,
            name="latch_tongue_reaches_keeper",
        )
        ctx.expect_overlap(
            panel,
            frame,
            axes="xy",
            min_overlap=0.40,
            elem_a=panel_shell,
            elem_b=bezel,
            name="door_covers_framed_opening",
        )

    with ctx.pose({hinge: 1.20}):
        ctx.expect_gap(
            panel,
            frame,
            axis="z",
            positive_elem=latch_bezel,
            negative_elem=bezel,
            min_gap=0.18,
            name="open_pose_swings_latch_side_outward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
