from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


VANE_LEVELS = (0.115, 0.210, 0.305, 0.400, 0.495)


def _vane_shell_mesh(
    *,
    length: float = 0.660,
    chord: float = 0.086,
    thickness: float = 0.012,
    camber: float = 0.006,
    pitch: float = math.radians(12.0),
) -> MeshGeometry:
    """A rounded, slightly cambered louver blade extruded along the pivot axis."""
    top = []
    bottom = []
    samples = 10
    for i in range(samples + 1):
        u = -1.0 + 2.0 * i / samples
        y = 0.5 * chord * u
        taper = max(0.0, 1.0 - u * u)
        z_camber = camber * (1.0 - 0.45 * u * u)
        half_t = 0.5 * thickness * (0.22 + 0.78 * math.sqrt(taper))
        top.append((y, z_camber + half_t))
        bottom.append((y, z_camber - half_t))

    profile = top + list(reversed(bottom))
    cos_p = math.cos(pitch)
    sin_p = math.sin(pitch)

    vertices = []
    for x in (-0.5 * length, 0.5 * length):
        for y, z in profile:
            # Rotate the chord slightly at the rest pose; later joint motion is
            # still about the long X pivot axis.
            yr = y * cos_p - z * sin_p
            zr = y * sin_p + z * cos_p
            vertices.append((x, yr, zr))

    n = len(profile)
    faces = []
    for j in range(n):
        a = j
        b = (j + 1) % n
        c = n + (j + 1) % n
        d = n + j
        faces.append((a, b, c))
        faces.append((a, c, d))

    left_center = len(vertices)
    vertices.append((-0.5 * length, 0.0, 0.0))
    right_center = len(vertices)
    vertices.append((0.5 * length, 0.0, 0.0))
    for j in range(n):
        faces.append((left_center, (j + 1) % n, j))
        faces.append((right_center, n + j, n + (j + 1) % n))

    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_vane_array")

    frame_mat = Material("powder_coated_rear_frame", color=(0.12, 0.14, 0.15, 1.0))
    bridge_mat = Material("slightly_lighter_bridge", color=(0.18, 0.20, 0.21, 1.0))
    bearing_mat = Material("dark_bronze_bushings", color=(0.33, 0.24, 0.13, 1.0))
    blade_mat = Material("satin_aluminum_blades", color=(0.72, 0.76, 0.78, 1.0))
    axle_mat = Material("polished_steel_axles", color=(0.82, 0.83, 0.80, 1.0))

    frame = model.part("rear_frame")

    # A rigid rear support frame: side rails, top/bottom rails, and a bridge
    # spine set behind the vanes so the array reads as bridge-backed rather than
    # just freestanding slats.
    frame.visual(Box((0.830, 0.080, 0.040)), origin=Origin(xyz=(0.0, 0.045, 0.020)), material=frame_mat, name="bottom_rail")
    frame.visual(Box((0.830, 0.080, 0.040)), origin=Origin(xyz=(0.0, 0.045, 0.590)), material=frame_mat, name="top_rail")
    for side, x in enumerate((-0.390, 0.390)):
        frame.visual(Box((0.040, 0.080, 0.610)), origin=Origin(xyz=(x, 0.045, 0.305)), material=frame_mat, name=f"side_rail_{side}")

    frame.visual(Box((0.030, 0.030, 0.535)), origin=Origin(xyz=(0.0, 0.077, 0.305)), material=bridge_mat, name="rear_spine")
    for idx, z in enumerate((0.165, 0.305, 0.445)):
        frame.visual(Box((0.765, 0.026, 0.018)), origin=Origin(xyz=(0.0, 0.077, z)), material=bridge_mat, name=f"bridge_bar_{idx}")

    # Each blade has its own pair of visible end bushings on the rear frame.
    for i, z in enumerate(VANE_LEVELS):
        for side, x in enumerate((-0.358, 0.358)):
            frame.visual(
                Cylinder(radius=0.019, length=0.028),
                origin=Origin(xyz=(x, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=bearing_mat,
                name=f"bearing_{i}_{side}",
            )

    vane_mesh = mesh_from_geometry(_vane_shell_mesh(), "cambered_vane_shell")
    for i, z in enumerate(VANE_LEVELS):
        vane = model.part(f"vane_{i}")
        vane.visual(vane_mesh, material=blade_mat, name="blade_shell")
        vane.visual(
            Cylinder(radius=0.0075, length=0.720),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_mat,
            name="axle",
        )
        model.articulation(
            f"vane_{i}_pivot",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=1.8, lower=-0.70, upper=0.70),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("rear_frame")
    joints = [object_model.get_articulation(f"vane_{i}_pivot") for i in range(len(VANE_LEVELS))]

    ctx.check(
        "five independent revolute vanes",
        len(joints) == 5
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints)
        and all(j.mimic is None for j in joints),
        details=f"joints={[j.name for j in joints]}",
    )
    ctx.check(
        "vane pivot axes are parallel",
        all(tuple(round(v, 6) for v in j.axis) == (1.0, 0.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    for i in range(len(VANE_LEVELS)):
        vane = object_model.get_part(f"vane_{i}")
        # The only intended interpenetration is the steel axle captured inside
        # the pair of frame bushings that carry that blade's revolute joint.
        for side in range(2):
            bearing = f"bearing_{i}_{side}"
            ctx.allow_overlap(
                vane,
                frame,
                elem_a="axle",
                elem_b=bearing,
                reason="The vane axle is intentionally captured inside its end bushing.",
            )
            ctx.expect_within(
                vane,
                frame,
                axes="yz",
                inner_elem="axle",
                outer_elem=bearing,
                margin=0.001,
                name=f"vane {i} axle centered in bushing {side}",
            )
            ctx.expect_overlap(
                vane,
                frame,
                axes="x",
                elem_a="axle",
                elem_b=bearing,
                min_overlap=0.010,
                name=f"vane {i} axle inserted in bushing {side}",
            )

    def _z_size(aabb):
        if aabb is None:
            return None
        return aabb[1][2] - aabb[0][2]

    rest_size = _z_size(ctx.part_element_world_aabb("vane_0", elem="blade_shell"))
    neighbor_rest = ctx.part_element_world_aabb("vane_1", elem="blade_shell")
    with ctx.pose({joints[0]: 0.55}):
        moved_size = _z_size(ctx.part_element_world_aabb("vane_0", elem="blade_shell"))
        neighbor_pose = ctx.part_element_world_aabb("vane_1", elem="blade_shell")
    ctx.check(
        "one vane can rotate without driving its neighbor",
        rest_size is not None
        and moved_size is not None
        and abs(moved_size - rest_size) > 0.010
        and neighbor_rest == neighbor_pose,
        details=f"vane0_z_size_rest={rest_size}, vane0_z_size_moved={moved_size}",
    )

    return ctx.report()


object_model = build_object_model()
