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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="standard_zoom_lens")

    alloy_black = model.material("alloy_black", rgba=(0.13, 0.13, 0.14, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.22, 0.22, 0.24, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.14, 0.20, 0.24, 0.78))
    index_paint = model.material("index_paint", rgba=(0.92, 0.92, 0.94, 1.0))

    def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
        merged = MeshGeometry()
        for geometry in geometries:
            merged.merge(geometry)
        return merged

    def _tube_shell_mesh(
        *,
        inner_radius: float,
        outer_radius: float,
        z0: float,
        z1: float,
        segments: int = 72,
    ) -> MeshGeometry:
        height = z1 - z0
        shell = boolean_difference(
            CylinderGeometry(radius=outer_radius, height=height, radial_segments=segments),
            CylinderGeometry(
                radius=inner_radius,
                height=height + 0.0008,
                radial_segments=segments,
            ),
        )
        shell.translate(0.0, 0.0, 0.5 * (z0 + z1))
        return shell

    def _ribbed_ring_mesh(
        *,
        inner_radius: float,
        outer_radius: float,
        length: float,
        rib_depth: float,
        rib_width: float,
        rib_count: int,
    ) -> MeshGeometry:
        shell = _tube_shell_mesh(
            inner_radius=inner_radius,
            outer_radius=outer_radius,
            z0=0.0,
            z1=length,
            segments=72,
        )
        rib_radius = outer_radius + (0.5 * rib_depth) - 0.00035
        rib_length = length - 0.002
        for index in range(rib_count):
            angle = (math.tau * index) / rib_count
            rib = BoxGeometry((rib_depth, rib_width, rib_length))
            rib.translate(rib_radius, 0.0, length * 0.5)
            rib.rotate_z(angle)
            shell.merge(rib)
        return shell

    body_shell_mesh = mesh_from_geometry(
        _merge_meshes(
            _tube_shell_mesh(inner_radius=0.0260, outer_radius=0.0375, z0=0.0040, z1=0.0180),
            _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0390, z0=0.0180, z1=0.0190),
            _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0402, z0=0.0190, z1=0.0200),
            _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0392, z0=0.0200, z1=0.0300),
            _tube_shell_mesh(inner_radius=0.0344, outer_radius=0.0392, z0=0.0300, z1=0.0450),
            _tube_shell_mesh(inner_radius=0.0344, outer_radius=0.0402, z0=0.0450, z1=0.0460),
            _tube_shell_mesh(inner_radius=0.0344, outer_radius=0.0374, z0=0.0460, z1=0.0580),
        ),
        "lens_body_shell",
    )
    mount_flange_mesh = mesh_from_geometry(
        _tube_shell_mesh(inner_radius=0.023, outer_radius=0.0345, z0=0.0, z1=0.004),
        "lens_mount_flange",
    )
    front_trim_mesh = mesh_from_geometry(
        _tube_shell_mesh(inner_radius=0.0344, outer_radius=0.0382, z0=0.0580, z1=0.0620),
        "lens_body_front_trim",
    )
    zoom_ring_mesh = mesh_from_geometry(
        _ribbed_ring_mesh(
            inner_radius=0.0402,
            outer_radius=0.0434,
            length=0.0270,
            rib_depth=0.0024,
            rib_width=0.0025,
            rib_count=36,
        ),
        "zoom_ring_shell",
    )
    inner_barrel_mesh = mesh_from_geometry(
        _merge_meshes(
            _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0332, z0=0.0000, z1=0.0280),
            _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0344, z0=0.0010, z1=0.0020),
            _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0344, z0=0.0270, z1=0.0280),
            _tube_shell_mesh(inner_radius=0.0292, outer_radius=0.0315, z0=0.0280, z1=0.0315),
            _tube_shell_mesh(inner_radius=0.0292, outer_radius=0.0326, z0=0.0315, z1=0.0325),
            _tube_shell_mesh(inner_radius=0.0292, outer_radius=0.0315, z0=0.0325, z1=0.0495),
            _tube_shell_mesh(inner_radius=0.0292, outer_radius=0.0326, z0=0.0495, z1=0.0505),
            _tube_shell_mesh(inner_radius=0.0292, outer_radius=0.0340, z0=0.0515, z1=0.0600),
        ),
        "inner_barrel_shell",
    )
    filter_rim_mesh = mesh_from_geometry(
        _tube_shell_mesh(inner_radius=0.0310, outer_radius=0.0380, z0=0.0600, z1=0.0660),
        "front_filter_rim",
    )
    front_retainer_mesh = mesh_from_geometry(
        _tube_shell_mesh(inner_radius=0.0285, outer_radius=0.0310, z0=0.0600, z1=0.0610),
        "front_glass_retainer",
    )
    focus_ring_mesh = mesh_from_geometry(
        _ribbed_ring_mesh(
            inner_radius=0.0326,
            outer_radius=0.0350,
            length=0.0200,
            rib_depth=0.0022,
            rib_width=0.0022,
            rib_count=30,
        ),
        "focus_ring_shell",
    )

    body = model.part("body")
    body.visual(body_shell_mesh, material=alloy_black, name="body_shell")
    body.visual(mount_flange_mesh, material=satin_metal, name="mount_flange")
    body.visual(front_trim_mesh, material=dark_trim, name="front_trim")
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0415, length=0.082),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(zoom_ring_mesh, material=rubber_black, name="zoom_ring_shell")
    zoom_ring.visual(
        Box((0.0018, 0.0055, 0.0030)),
        origin=Origin(xyz=(0.0439, 0.0, 0.0135)),
        material=index_paint,
        name="zoom_marker",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.032),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(inner_barrel_mesh, material=alloy_black, name="inner_barrel_shell")
    inner_barrel.visual(filter_rim_mesh, material=dark_trim, name="front_filter_rim")
    inner_barrel.visual(front_retainer_mesh, material=dark_trim, name="front_glass_retainer")
    inner_barrel.visual(
        Cylinder(radius=0.0285, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
        material=coated_glass,
        name="front_glass",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.055),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(focus_ring_mesh, material=rubber_black, name="focus_ring_shell")
    focus_ring.visual(
        Box((0.0018, 0.0048, 0.0028)),
        origin=Origin(xyz=(0.0356, 0.0, 0.0100)),
        material=index_paint,
        name="focus_marker",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.021),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.0105)),
    )

    model.articulation(
        "body_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.72,
            upper=0.72,
        ),
    )
    model.articulation(
        "body_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=body,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=0.0,
            upper=0.028,
        ),
    )
    model.articulation(
        "inner_barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0315)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=3.5,
            lower=-1.35,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    zoom_ring = object_model.get_part("zoom_ring")
    inner_barrel = object_model.get_part("inner_barrel")
    focus_ring = object_model.get_part("focus_ring")

    zoom_joint = object_model.get_articulation("body_to_zoom_ring")
    extension_joint = object_model.get_articulation("body_to_inner_barrel")
    focus_joint = object_model.get_articulation("inner_barrel_to_focus_ring")

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        body,
        zoom_ring,
        elem_a="body_shell",
        elem_b="zoom_ring_shell",
        reason=(
            "The zoom sleeve is authored as a tight concentric shell around the body carrier; "
            "the SDK overlap gate reads this nested annular interface as overlap."
        ),
    )
    ctx.allow_overlap(
        body,
        inner_barrel,
        elem_a="body_shell",
        elem_b="inner_barrel_shell",
        reason=(
            "The extending inner barrel rides inside the main barrel on a nested cylindrical guide "
            "surface that the shell-overlap sensor flags as penetration."
        ),
    )
    ctx.allow_overlap(
        focus_ring,
        inner_barrel,
        elem_a="focus_ring_shell",
        elem_b="inner_barrel_shell",
        reason=(
            "The focus grip is a concentric shell over the front barrel guide surface, with the "
            "same nested-shell false-positive overlap behavior."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(zoom_ring, body, name="zoom_ring_mounted_to_body")
    ctx.expect_contact(inner_barrel, body, name="inner_barrel_guided_in_body")
    ctx.expect_contact(focus_ring, inner_barrel, name="focus_ring_mounted_to_inner_barrel")

    ctx.expect_overlap(zoom_ring, body, axes="xy", min_overlap=0.078, name="zoom_ring_wraps_body")
    ctx.expect_overlap(inner_barrel, body, axes="xy", min_overlap=0.056, name="inner_barrel_stays_coaxial")
    ctx.expect_overlap(
        focus_ring,
        inner_barrel,
        axes="xy",
        min_overlap=0.060,
        name="focus_ring_wraps_inner_barrel",
    )

    ctx.check(
        "zoom_joint_axis",
        tuple(zoom_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Unexpected zoom axis: {zoom_joint.axis}",
    )
    ctx.check(
        "extension_joint_axis",
        tuple(extension_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Unexpected extension axis: {extension_joint.axis}",
    )
    ctx.check(
        "focus_joint_axis",
        tuple(focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"Unexpected focus axis: {focus_joint.axis}",
    )

    zoom_marker_rest = ctx.part_element_world_aabb(zoom_ring, elem="zoom_marker")
    assert zoom_marker_rest is not None
    zoom_marker_rest_center = _aabb_center(zoom_marker_rest)
    with ctx.pose({zoom_joint: 0.55}):
        zoom_marker_turned = ctx.part_element_world_aabb(zoom_ring, elem="zoom_marker")
        assert zoom_marker_turned is not None
        zoom_marker_turned_center = _aabb_center(zoom_marker_turned)
        ctx.check(
            "zoom_ring_rotates",
            zoom_marker_turned_center[1] > zoom_marker_rest_center[1] + 0.015
            and zoom_marker_turned_center[0] < zoom_marker_rest_center[0] - 0.004,
            details=(
                f"Zoom marker did not sweep around optical axis: "
                f"rest={zoom_marker_rest_center}, turned={zoom_marker_turned_center}"
            ),
        )
        ctx.expect_contact(zoom_ring, body, name="zoom_ring_keeps_body_contact_when_rotated")

    focus_marker_rest = ctx.part_element_world_aabb(focus_ring, elem="focus_marker")
    assert focus_marker_rest is not None
    focus_marker_rest_center = _aabb_center(focus_marker_rest)
    with ctx.pose({focus_joint: 0.85}):
        focus_marker_turned = ctx.part_element_world_aabb(focus_ring, elem="focus_marker")
        assert focus_marker_turned is not None
        focus_marker_turned_center = _aabb_center(focus_marker_turned)
        ctx.check(
            "focus_ring_rotates",
            focus_marker_turned_center[1] > focus_marker_rest_center[1] + 0.018
            and focus_marker_turned_center[0] < focus_marker_rest_center[0] - 0.008,
            details=(
                f"Focus marker did not sweep around optical axis: "
                f"rest={focus_marker_rest_center}, turned={focus_marker_turned_center}"
            ),
        )
        ctx.expect_contact(focus_ring, inner_barrel, name="focus_ring_keeps_barrel_contact_when_rotated")

    inner_barrel_rest = ctx.part_world_aabb(inner_barrel)
    assert inner_barrel_rest is not None
    with ctx.pose({extension_joint: 0.026}):
        inner_barrel_extended = ctx.part_world_aabb(inner_barrel)
        assert inner_barrel_extended is not None
        ctx.check(
            "inner_barrel_extends_forward",
            inner_barrel_extended[1][2] > inner_barrel_rest[1][2] + 0.020
            and inner_barrel_extended[0][2] > inner_barrel_rest[0][2] + 0.020,
            details=(
                f"Inner barrel did not translate forward enough: "
                f"rest={inner_barrel_rest}, extended={inner_barrel_extended}"
            ),
        )
        ctx.expect_contact(inner_barrel, body, name="inner_barrel_stays_guided_when_extended")

    with ctx.pose({zoom_joint: 0.55, extension_joint: 0.028, focus_joint: -0.9}):
        ctx.expect_contact(zoom_ring, body, name="zoom_ring_contact_at_full_extension")
        ctx.expect_contact(inner_barrel, body, name="inner_barrel_contact_at_full_extension")
        ctx.expect_contact(focus_ring, inner_barrel, name="focus_ring_contact_at_full_extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
