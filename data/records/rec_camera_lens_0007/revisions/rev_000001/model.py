from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _shell_mesh(
    filename: str,
    *,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    segments: int = 72,
):
    return _save_mesh(
        filename,
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
    )


def _ribbed_sleeve_mesh(
    filename: str,
    *,
    inner_radius: float,
    base_outer_radius: float,
    peak_outer_radius: float,
    z_start: float,
    z_end: float,
    rib_count: int,
    segments: int = 84,
):
    pitch = (z_end - z_start) / rib_count
    outer_profile = [(base_outer_radius, z_start)]
    for rib_index in range(rib_count):
        base_z = z_start + rib_index * pitch
        outer_profile.extend(
            [
                (base_outer_radius, base_z + 0.14 * pitch),
                (peak_outer_radius, base_z + 0.36 * pitch),
                (base_outer_radius, base_z + 0.72 * pitch),
            ]
        )
    outer_profile.append((base_outer_radius, z_end))
    inner_profile = [(inner_radius, z_pos) for _, z_pos in outer_profile]
    return _shell_mesh(
        filename,
        outer_profile=outer_profile,
        inner_profile=inner_profile,
        segments=segments,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wide_angle_zoom_lens", assets=ASSETS)

    anodized_black = model.material("anodized_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_black = model.material("satin_black", rgba=(0.20, 0.20, 0.22, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.16, 0.24, 0.30, 0.45))
    pale_mark = model.material("pale_mark", rgba=(0.84, 0.86, 0.88, 1.0))

    body = model.part("body")
    body.visual(
        _shell_mesh(
            "outer_body_shell.obj",
            outer_profile=[
                (0.0455, -0.034),
                (0.0455, -0.028),
                (0.0435, -0.022),
                (0.0435, -0.016),
                (0.0510, -0.015),
                (0.0510, 0.015),
                (0.0460, 0.019),
                (0.0430, 0.026),
                (0.0420, 0.029),
            ],
            inner_profile=[
                (0.0310, -0.034),
                (0.0310, -0.026),
                (0.0340, -0.018),
                (0.0340, 0.024),
                (0.0335, 0.028),
                (0.0325, 0.029),
            ],
        ),
        material=anodized_black,
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.049, length=0.063),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.0025)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _ribbed_sleeve_mesh(
            "zoom_ring_shell.obj",
            inner_radius=0.0510,
            base_outer_radius=0.0545,
            peak_outer_radius=0.0562,
            z_start=-0.014,
            z_end=0.014,
            rib_count=11,
        ),
        material=rubber,
        name="zoom_ring_shell",
    )
    zoom_ring.visual(
        Box((0.0070, 0.0018, 0.0050)),
        origin=Origin(xyz=(0.0, 0.0567, 0.0)),
        material=pale_mark,
        name="zoom_marker",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0562, length=0.028),
        mass=0.08,
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _shell_mesh(
            "inner_barrel_shell.obj",
            outer_profile=[
                (0.0340, -0.018),
                (0.0340, 0.022),
                (0.0387, 0.032),
                (0.0387, 0.050),
                (0.0362, 0.058),
            ],
            inner_profile=[
                (0.0280, -0.018),
                (0.0280, 0.022),
                (0.0270, 0.032),
                (0.0255, 0.050),
                (0.0255, 0.058),
            ],
        ),
        material=anodized_black,
        name="inner_barrel_shell",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0255, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=lens_glass,
        name="front_glass",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.041, length=0.076),
        mass=0.19,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _ribbed_sleeve_mesh(
            "focus_ring_shell.obj",
            inner_radius=0.0387,
            base_outer_radius=0.0422,
            peak_outer_radius=0.0433,
            z_start=-0.009,
            z_end=0.009,
            rib_count=9,
        ),
        material=rubber,
        name="focus_ring_shell",
    )
    focus_ring.visual(
        Box((0.0055, 0.0016, 0.0038)),
        origin=Origin(xyz=(0.0, 0.0440, 0.0)),
        material=pale_mark,
        name="focus_marker",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0433, length=0.018),
        mass=0.05,
    )

    model.articulation(
        "body_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=body,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.2,
            lower=-0.65,
            upper=0.65,
        ),
    )
    model.articulation(
        "body_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=body,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "inner_barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=inner_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=-1.0,
            upper=1.0,
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

    body_shell = body.get_visual("body_shell")
    zoom_shell = zoom_ring.get_visual("zoom_ring_shell")
    zoom_marker = zoom_ring.get_visual("zoom_marker")
    inner_shell = inner_barrel.get_visual("inner_barrel_shell")
    front_glass = inner_barrel.get_visual("front_glass")
    focus_shell = focus_ring.get_visual("focus_ring_shell")
    focus_marker = focus_ring.get_visual("focus_marker")

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        body,
        zoom_ring,
        reason="The zoom ring is a concentric rubber sleeve that intentionally wraps the outer barrel with a tiny running clearance that the hollow-shell overlap sensor reads as nested overlap.",
    )
    ctx.allow_overlap(
        body,
        inner_barrel,
        reason="The extending optical barrel telescopes inside the hollow outer body; this is intentional nesting, not solid interpenetration.",
    )
    ctx.allow_overlap(
        inner_barrel,
        focus_ring,
        reason="The focus ring is another concentric sleeve riding on the inner barrel, so their hollow shell volumes are intentionally nested.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=64,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.check(
        "zoom ring joint axis",
        zoom_joint.axis == (0.0, 0.0, 1.0),
        f"expected zoom ring to rotate about +Z, got {zoom_joint.axis}",
    )
    ctx.check(
        "inner barrel extension axis",
        extension_joint.axis == (0.0, 0.0, 1.0),
        f"expected inner barrel to slide on +Z, got {extension_joint.axis}",
    )
    ctx.check(
        "focus ring joint axis",
        focus_joint.axis == (0.0, 0.0, 1.0),
        f"expected focus ring to rotate about +Z, got {focus_joint.axis}",
    )

    body_aabb = ctx.part_world_aabb(body)
    ctx.check(
        "squat body proportions",
        body_aabb is not None
        and (body_aabb[1][0] - body_aabb[0][0]) > 0.09
        and (body_aabb[1][2] - body_aabb[0][2]) < 0.075,
        f"body AABB did not read as a short squat lens barrel: {body_aabb}",
    )

    ctx.expect_origin_distance(zoom_ring, body, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(inner_barrel, body, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(focus_ring, inner_barrel, axes="xy", max_dist=0.001)

    ctx.expect_overlap(zoom_ring, body, axes="xy", min_overlap=0.094, elem_a=zoom_shell, elem_b=body_shell)
    ctx.expect_overlap(zoom_ring, body, axes="z", min_overlap=0.026, elem_a=zoom_shell, elem_b=body_shell)
    ctx.expect_overlap(focus_ring, inner_barrel, axes="xy", min_overlap=0.067, elem_a=focus_shell, elem_b=inner_shell)
    ctx.expect_overlap(focus_ring, inner_barrel, axes="z", min_overlap=0.017, elem_a=focus_shell, elem_b=inner_shell)
    ctx.expect_within(inner_barrel, body, axes="xy", inner_elem=inner_shell, outer_elem=body_shell)
    ctx.expect_within(inner_barrel, focus_ring, axes="xy", inner_elem=inner_shell, outer_elem=focus_shell)
    ctx.expect_gap(
        focus_ring,
        body,
        axis="z",
        min_gap=0.0015,
        max_gap=0.0045,
        positive_elem=focus_shell,
        negative_elem=body_shell,
        name="focus ring starts just ahead of front barrel",
    )
    ctx.expect_gap(
        inner_barrel,
        body,
        axis="z",
        min_gap=0.016,
        positive_elem=front_glass,
        negative_elem=body_shell,
        name="front optical group protrudes beyond squat body",
    )

    zoom_marker_rest_aabb = ctx.part_element_world_aabb(zoom_ring, elem=zoom_marker)
    focus_marker_rest_aabb = ctx.part_element_world_aabb(focus_ring, elem=focus_marker)
    inner_rest_pos = ctx.part_world_position(inner_barrel)
    focus_rest_pos = ctx.part_world_position(focus_ring)
    assert zoom_marker_rest_aabb is not None
    assert focus_marker_rest_aabb is not None
    assert inner_rest_pos is not None
    assert focus_rest_pos is not None

    zoom_limits = zoom_joint.motion_limits
    extension_limits = extension_joint.motion_limits
    focus_limits = focus_joint.motion_limits
    assert zoom_limits is not None
    assert extension_limits is not None
    assert focus_limits is not None
    assert zoom_limits.upper is not None
    assert extension_limits.upper is not None
    assert focus_limits.upper is not None
    assert zoom_limits.lower is not None
    assert extension_limits.lower is not None
    assert focus_limits.lower is not None

    with ctx.pose({zoom_joint: zoom_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="zoom_ring_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="zoom_ring_lower_no_floating")
    with ctx.pose({zoom_joint: zoom_limits.upper}):
        zoom_marker_rotated_aabb = ctx.part_element_world_aabb(zoom_ring, elem=zoom_marker)
        assert zoom_marker_rotated_aabb is not None
        zoom_marker_rest = _aabb_center(zoom_marker_rest_aabb)
        zoom_marker_rotated = _aabb_center(zoom_marker_rotated_aabb)
        ctx.check(
            "zoom ring marker rotates around barrel",
            zoom_marker_rotated[0] < zoom_marker_rest[0] - 0.02,
            f"zoom marker did not sweep tangentially enough: rest={zoom_marker_rest}, posed={zoom_marker_rotated}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="zoom_ring_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="zoom_ring_upper_no_floating")

    with ctx.pose({focus_joint: focus_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="focus_ring_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="focus_ring_lower_no_floating")
    with ctx.pose({focus_joint: focus_limits.upper}):
        focus_marker_rotated_aabb = ctx.part_element_world_aabb(focus_ring, elem=focus_marker)
        assert focus_marker_rotated_aabb is not None
        focus_marker_rest = _aabb_center(focus_marker_rest_aabb)
        focus_marker_rotated = _aabb_center(focus_marker_rotated_aabb)
        ctx.check(
            "focus ring marker rotates around the front inner barrel land",
            focus_marker_rotated[0] < focus_marker_rest[0] - 0.02,
            f"focus marker did not sweep tangentially enough: rest={focus_marker_rest}, posed={focus_marker_rotated}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="focus_ring_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="focus_ring_upper_no_floating")

    with ctx.pose({extension_joint: extension_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="inner_barrel_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="inner_barrel_lower_no_floating")
    with ctx.pose({extension_joint: extension_limits.upper}):
        inner_extended_pos = ctx.part_world_position(inner_barrel)
        focus_extended_pos = ctx.part_world_position(focus_ring)
        assert inner_extended_pos is not None
        assert focus_extended_pos is not None
        ctx.check(
            "inner barrel extends forward with zoom travel",
            inner_extended_pos[2] > inner_rest_pos[2] + 0.017,
            f"expected roughly 18 mm of extension, rest={inner_rest_pos}, extended={inner_extended_pos}",
        )
        ctx.check(
            "focus ring rides forward on the extending inner barrel",
            focus_extended_pos[2] > focus_rest_pos[2] + 0.017,
            f"focus ring did not translate with the inner barrel: rest={focus_rest_pos}, extended={focus_extended_pos}",
        )
        ctx.expect_gap(
            focus_ring,
            body,
            axis="z",
            min_gap=0.019,
            positive_elem=focus_shell,
            negative_elem=body_shell,
            name="focus ring stays forward of body when zoomed",
        )
        ctx.expect_gap(
            inner_barrel,
            body,
            axis="z",
            min_gap=0.034,
            positive_elem=front_glass,
            negative_elem=body_shell,
            name="front glass projects farther in zoomed pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="inner_barrel_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="inner_barrel_upper_no_floating")

    with ctx.pose(
        {
            zoom_joint: zoom_limits.upper,
            extension_joint: extension_limits.upper,
            focus_joint: focus_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
