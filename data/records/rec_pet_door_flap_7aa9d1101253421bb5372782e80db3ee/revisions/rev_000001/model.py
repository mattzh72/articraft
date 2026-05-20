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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_flap(
    model: ArticulatedObject,
    *,
    name: str,
    flap_width: float,
    flap_height: float,
    flap_thickness: float,
    border: float,
    hinge_radius: float,
    flap_frame_material,
    pane_material,
    magnet_material,
) -> None:
    flap = model.part(name)
    flap.inertial = Inertial.from_geometry(
        Box((flap_width, flap_thickness, flap_height)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -flap_height * 0.5)),
    )

    clear_width = flap_width - 2.0 * border
    clear_height = flap_height - 2.0 * border
    side_height = flap_height - 2.0 * border
    bottom_center_z = -(flap_height - border * 0.5)
    middle_center_z = -(border + clear_height * 0.5)

    flap.visual(
        Cylinder(radius=hinge_radius, length=flap_width - 0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=flap_frame_material,
        name=f"{name}_hinge_tube",
    )
    flap.visual(
        Box((flap_width, flap_thickness, border)),
        origin=Origin(xyz=(0.0, 0.0, -border * 0.5)),
        material=flap_frame_material,
        name=f"{name}_top_rail",
    )
    flap.visual(
        Box((border, flap_thickness, side_height)),
        origin=Origin(xyz=(-(flap_width * 0.5 - border * 0.5), 0.0, -flap_height * 0.5)),
        material=flap_frame_material,
        name=f"{name}_left_stile",
    )
    flap.visual(
        Box((border, flap_thickness, side_height)),
        origin=Origin(xyz=(flap_width * 0.5 - border * 0.5, 0.0, -flap_height * 0.5)),
        material=flap_frame_material,
        name=f"{name}_right_stile",
    )
    flap.visual(
        Box((flap_width, flap_thickness, border)),
        origin=Origin(xyz=(0.0, 0.0, bottom_center_z)),
        material=flap_frame_material,
        name=f"{name}_bottom_rail",
    )
    flap.visual(
        Box((clear_width, flap_thickness * 0.55, clear_height)),
        origin=Origin(xyz=(0.0, 0.0, middle_center_z)),
        material=pane_material,
        name=f"{name}_pane",
    )
    flap.visual(
        Box((flap_width * 0.42, flap_thickness * 0.9, border * 0.55)),
        origin=Origin(xyz=(0.0, 0.0, bottom_center_z)),
        material=magnet_material,
        name=f"{name}_magnet_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="draft_reducing_pet_door")

    frame_white = model.material("frame_white", rgba=(0.93, 0.93, 0.92, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.72, 0.73, 0.74, 1.0))
    flap_frame = model.material("flap_frame", rgba=(0.14, 0.15, 0.16, 1.0))
    tunnel_liner = model.material("tunnel_liner", rgba=(0.83, 0.84, 0.85, 1.0))
    smoked_pane = model.material("smoked_pane", rgba=(0.28, 0.34, 0.40, 0.45))
    magnet_black = model.material("magnet_black", rgba=(0.09, 0.09, 0.10, 1.0))

    outer_width = 0.40
    outer_height = 0.50
    frame_depth = 0.20
    opening_width = 0.26
    opening_height = 0.34
    border_x = (outer_width - opening_width) * 0.5
    border_z = (outer_height - opening_height) * 0.5
    bezel_thickness = 0.014
    tunnel_depth = frame_depth - 2.0 * bezel_thickness

    opening_bottom = border_z
    opening_top = opening_bottom + opening_height
    hinge_axis_z = opening_top - 0.009

    outer_flap_plane_y = -(frame_depth * 0.5 - bezel_thickness - 0.010)
    inner_flap_plane_y = frame_depth * 0.5 - bezel_thickness - 0.010

    flap_width = opening_width - 0.012
    flap_height = opening_height - 0.015
    flap_thickness = 0.008
    flap_border = 0.018
    hinge_radius = 0.005

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, frame_depth, outer_height)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.0, outer_height * 0.5)),
    )

    for side, y in (("front", -(frame_depth * 0.5 - bezel_thickness * 0.5)), ("rear", frame_depth * 0.5 - bezel_thickness * 0.5)):
        frame.visual(
            Box((border_x, bezel_thickness, outer_height)),
            origin=Origin(xyz=(-(opening_width * 0.5 + border_x * 0.5), y, outer_height * 0.5)),
            material=frame_white,
            name=f"{side}_left_bezel",
        )
        frame.visual(
            Box((border_x, bezel_thickness, outer_height)),
            origin=Origin(xyz=(opening_width * 0.5 + border_x * 0.5, y, outer_height * 0.5)),
            material=frame_white,
            name=f"{side}_right_bezel",
        )
        frame.visual(
            Box((opening_width, bezel_thickness, border_z)),
            origin=Origin(xyz=(0.0, y, outer_height - border_z * 0.5)),
            material=frame_gray if side == "rear" else frame_white,
            name=f"{side}_top_bezel",
        )
        frame.visual(
            Box((opening_width, bezel_thickness, border_z)),
            origin=Origin(xyz=(0.0, y, border_z * 0.5)),
            material=frame_gray if side == "rear" else frame_white,
            name=f"{side}_bottom_bezel",
        )

    frame.visual(
        Box((border_x, tunnel_depth, opening_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + border_x * 0.5), 0.0, opening_bottom + opening_height * 0.5)),
        material=tunnel_liner,
        name="left_tunnel_wall",
    )
    frame.visual(
        Box((border_x, tunnel_depth, opening_height)),
        origin=Origin(xyz=(opening_width * 0.5 + border_x * 0.5, 0.0, opening_bottom + opening_height * 0.5)),
        material=tunnel_liner,
        name="right_tunnel_wall",
    )
    frame.visual(
        Box((opening_width, tunnel_depth, border_z)),
        origin=Origin(xyz=(0.0, 0.0, outer_height - border_z * 0.5)),
        material=tunnel_liner,
        name="top_tunnel_wall",
    )
    frame.visual(
        Box((opening_width, tunnel_depth, border_z)),
        origin=Origin(xyz=(0.0, 0.0, border_z * 0.5)),
        material=tunnel_liner,
        name="tunnel_sill",
    )
    frame.visual(
        Box((opening_width - 0.020, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, outer_flap_plane_y, hinge_axis_z + hinge_radius + 0.005)),
        material=flap_frame,
        name="outer_header",
    )
    frame.visual(
        Box((opening_width - 0.020, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, inner_flap_plane_y, hinge_axis_z + hinge_radius + 0.005)),
        material=flap_frame,
        name="inner_header",
    )
    frame.visual(
        Box((opening_width - 0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, outer_flap_plane_y, opening_bottom + 0.005)),
        material=magnet_black,
        name="outer_latch_strip",
    )
    frame.visual(
        Box((opening_width - 0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, inner_flap_plane_y, opening_bottom + 0.005)),
        material=magnet_black,
        name="inner_latch_strip",
    )

    _add_flap(
        model,
        name="outer_flap",
        flap_width=flap_width,
        flap_height=flap_height,
        flap_thickness=flap_thickness,
        border=flap_border,
        hinge_radius=hinge_radius,
        flap_frame_material=flap_frame,
        pane_material=smoked_pane,
        magnet_material=magnet_black,
    )
    _add_flap(
        model,
        name="inner_flap",
        flap_width=flap_width,
        flap_height=flap_height,
        flap_thickness=flap_thickness,
        border=flap_border,
        hinge_radius=hinge_radius,
        flap_frame_material=flap_frame,
        pane_material=smoked_pane,
        magnet_material=magnet_black,
    )

    model.articulation(
        "outer_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="outer_flap",
        origin=Origin(xyz=(0.0, outer_flap_plane_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-1.0, upper=1.0),
    )
    model.articulation(
        "inner_flap_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child="inner_flap",
        origin=Origin(xyz=(0.0, inner_flap_plane_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=-1.0, upper=1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    outer_flap = object_model.get_part("outer_flap")
    inner_flap = object_model.get_part("inner_flap")
    outer_hinge = object_model.get_articulation("outer_flap_hinge")
    inner_hinge = object_model.get_articulation("inner_flap_hinge")

    outer_header = frame.get_visual("outer_header")
    inner_header = frame.get_visual("inner_header")
    sill = frame.get_visual("tunnel_sill")
    outer_hinge_tube = outer_flap.get_visual("outer_flap_hinge_tube")
    inner_hinge_tube = inner_flap.get_visual("inner_flap_hinge_tube")
    outer_bottom_rail = outer_flap.get_visual("outer_flap_bottom_rail")
    inner_bottom_rail = inner_flap.get_visual("inner_flap_bottom_rail")

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

    ctx.check(
        "outer hinge rotates on horizontal upper axis",
        tuple(round(value, 6) for value in outer_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"outer hinge axis was {outer_hinge.axis}",
    )
    ctx.check(
        "inner hinge rotates on horizontal upper axis",
        tuple(round(value, 6) for value in inner_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"inner hinge axis was {inner_hinge.axis}",
    )
    ctx.check(
        "two hinge axes are separated through tunnel depth",
        abs(inner_hinge.origin.xyz[1] - outer_hinge.origin.xyz[1]) >= 0.15,
        details=f"hinge y positions were {outer_hinge.origin.xyz[1]} and {inner_hinge.origin.xyz[1]}",
    )

    ctx.expect_contact(
        outer_flap,
        frame,
        elem_a=outer_hinge_tube,
        elem_b=outer_header,
        contact_tol=1e-5,
        name="outer flap hangs from outer header",
    )
    ctx.expect_contact(
        inner_flap,
        frame,
        elem_a=inner_hinge_tube,
        elem_b=inner_header,
        contact_tol=1e-5,
        name="inner flap hangs from inner header",
    )
    ctx.expect_gap(
        inner_flap,
        outer_flap,
        axis="y",
        min_gap=0.14,
        max_gap=0.16,
        name="two flaps occupy distinct planes in the tunnel",
    )
    ctx.expect_overlap(
        outer_flap,
        inner_flap,
        axes="xz",
        min_overlap=0.20,
        name="two flaps cover the same passage opening",
    )
    ctx.expect_gap(
        outer_flap,
        frame,
        axis="z",
        min_gap=0.006,
        max_gap=0.012,
        positive_elem=outer_bottom_rail,
        negative_elem=sill,
        name="outer flap hangs just above sill",
    )
    ctx.expect_gap(
        inner_flap,
        frame,
        axis="z",
        min_gap=0.006,
        max_gap=0.012,
        positive_elem=inner_bottom_rail,
        negative_elem=sill,
        name="inner flap hangs just above sill",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
