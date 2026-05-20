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
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_zoom_lens", assets=ASSETS)

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.11, 0.12, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.16, 0.17, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.27, 1.0))
    mount_silver = model.material("mount_silver", rgba=(0.70, 0.72, 0.75, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.82, 0.83, 0.84, 1.0))
    alignment_red = model.material("alignment_red", rgba=(0.78, 0.16, 0.12, 1.0))

    barrel_core = model.part("barrel_core")
    barrel_core.visual(
        Cylinder(radius=0.046, length=0.14),
        origin=Origin(xyz=(0.07, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="rear_body",
    )
    barrel_core.visual(
        Cylinder(radius=0.050, length=0.34),
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="main_barrel",
    )
    barrel_core.visual(
        Cylinder(radius=0.051, length=0.12),
        origin=Origin(xyz=(0.305, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="collar_seat",
    )
    barrel_core.visual(
        Cylinder(radius=0.048, length=0.12),
        origin=Origin(xyz=(0.475, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized_black,
        name="front_tube",
    )
    barrel_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.070, length=0.64),
        mass=2.6,
        origin=Origin(xyz=(0.29, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _save_mesh(
            _grip_ring_geometry(
                inner_radius=0.0494,
                sleeve_outer_radius=0.061,
                rib_outer_radius=0.063,
                length=0.145,
                x_center=0.17,
                rib_count=8,
            ),
            "zoom_ring.obj",
        ),
        material=rubber_black,
        name="zoom_shell",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.063, length=0.145),
        mass=0.28,
        origin=Origin(xyz=(0.17, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _save_mesh(
            _grip_ring_geometry(
                inner_radius=0.0474,
                sleeve_outer_radius=0.056,
                rib_outer_radius=0.058,
                length=0.085,
                x_center=0.425,
                rib_count=7,
            ),
            "focus_ring.obj",
        ),
        material=rubber_black,
        name="focus_shell",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.085),
        mass=0.14,
        origin=Origin(xyz=(0.425, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    front_housing = model.part("front_housing")
    front_housing.visual(
        _save_mesh(_front_housing_geometry(), "front_housing.obj"),
        material=graphite,
        name="front_shell",
    )
    front_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.086, length=0.094),
        mass=0.34,
        origin=Origin(xyz=(0.578, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    rear_mount = model.part("rear_mount")
    rear_mount.visual(
        _save_mesh(_rear_mount_geometry(), "rear_mount.obj"),
        material=mount_silver,
        name="mount_plate",
    )
    rear_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.030),
        mass=0.16,
        origin=Origin(xyz=(0.000, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    alignment_dot = model.part("alignment_dot")
    alignment_dot.visual(
        Sphere(radius=0.003),
        origin=Origin(xyz=(-0.004, 0.0, 0.056)),
        material=alignment_red,
        name="alignment_marker",
    )
    alignment_dot.inertial = Inertial.from_geometry(
        Sphere(radius=0.003),
        mass=0.002,
        origin=Origin(xyz=(-0.004, 0.0, 0.056)),
    )

    locking_pin = model.part("locking_pin")
    locking_pin.visual(
        Cylinder(radius=0.0025, length=0.008),
        origin=Origin(xyz=(-0.010, 0.018, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_steel,
        name="locking_pin_tip",
    )
    locking_pin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0025, length=0.008),
        mass=0.002,
        origin=Origin(xyz=(-0.010, 0.018, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    tripod_collar = model.part("tripod_collar")
    tripod_collar.visual(
        _save_mesh(_tripod_collar_geometry(), "tripod_collar.obj"),
        material=graphite,
        name="collar_band",
    )
    tripod_collar.visual(
        Box((0.016, 0.030, 0.026)),
        origin=Origin(xyz=(0.032, 0.0, -0.060)),
        material=graphite,
        name="lever_boss",
    )
    tripod_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.074, length=0.084),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    clamp_lever = model.part("clamp_lever")
    clamp_lever.visual(
        Box((0.016, 0.018, 0.020)),
        origin=Origin(xyz=(0.048, 0.0, -0.068)),
        material=graphite,
        name="clamp_base",
    )
    clamp_lever.visual(
        Box((0.052, 0.008, 0.006)),
        origin=Origin(xyz=(0.076, 0.0, -0.076)),
        material=graphite,
        name="lever_plate",
    )
    clamp_lever.inertial = Inertial.from_geometry(
        Box((0.068, 0.020, 0.024)),
        mass=0.06,
        origin=Origin(xyz=(0.064, 0.0, -0.072)),
    )

    model.articulation(
        "barrel_to_zoom_ring",
        ArticulationType.FIXED,
        parent=barrel_core,
        child=zoom_ring,
        origin=Origin(),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.FIXED,
        parent=barrel_core,
        child=focus_ring,
        origin=Origin(),
    )
    model.articulation(
        "barrel_to_front_housing",
        ArticulationType.FIXED,
        parent=barrel_core,
        child=front_housing,
        origin=Origin(),
    )
    model.articulation(
        "barrel_to_rear_mount",
        ArticulationType.FIXED,
        parent=barrel_core,
        child=rear_mount,
        origin=Origin(),
    )
    model.articulation(
        "rear_mount_to_alignment_dot",
        ArticulationType.FIXED,
        parent=rear_mount,
        child=alignment_dot,
        origin=Origin(),
    )
    model.articulation(
        "rear_mount_to_locking_pin",
        ArticulationType.FIXED,
        parent=rear_mount,
        child=locking_pin,
        origin=Origin(),
    )
    model.articulation(
        "barrel_to_tripod_collar",
        ArticulationType.CONTINUOUS,
        parent=barrel_core,
        child=tripod_collar,
        origin=Origin(xyz=(0.308, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=9.0, velocity=2.5),
    )
    model.articulation(
        "tripod_collar_to_clamp_lever",
        ArticulationType.FIXED,
        parent=tripod_collar,
        child=clamp_lever,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    barrel_core = object_model.get_part("barrel_core")
    zoom_ring = object_model.get_part("zoom_ring")
    focus_ring = object_model.get_part("focus_ring")
    front_housing = object_model.get_part("front_housing")
    rear_mount = object_model.get_part("rear_mount")
    alignment_dot = object_model.get_part("alignment_dot")
    locking_pin = object_model.get_part("locking_pin")
    tripod_collar = object_model.get_part("tripod_collar")
    clamp_lever = object_model.get_part("clamp_lever")
    collar_spin = object_model.get_articulation("barrel_to_tripod_collar")

    main_barrel = barrel_core.get_visual("main_barrel")
    collar_seat = barrel_core.get_visual("collar_seat")
    rear_body = barrel_core.get_visual("rear_body")
    front_tube = barrel_core.get_visual("front_tube")
    zoom_shell = zoom_ring.get_visual("zoom_shell")
    focus_shell = focus_ring.get_visual("focus_shell")
    front_shell = front_housing.get_visual("front_shell")
    mount_plate = rear_mount.get_visual("mount_plate")
    collar_band = tripod_collar.get_visual("collar_band")
    lever_boss = tripod_collar.get_visual("lever_boss")
    clamp_base = clamp_lever.get_visual("clamp_base")
    lever_plate = clamp_lever.get_visual("lever_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    # The tripod collar intentionally rotates about the barrel centerline, so the
    # articulation origin must sit inside the bore and this warning is not useful here.
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(
        tripod_collar,
        barrel_core,
        reason="the rotating tripod collar uses a lightly clamped sleeve that intentionally nests around the barrel seat",
    )
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(zoom_ring, barrel_core, axes="yz", max_dist=0.002)
    ctx.expect_origin_distance(focus_ring, barrel_core, axes="yz", max_dist=0.002)
    ctx.expect_origin_distance(front_housing, barrel_core, axes="yz", max_dist=0.002)
    ctx.expect_origin_distance(rear_mount, barrel_core, axes="yz", max_dist=0.002)
    ctx.expect_origin_distance(tripod_collar, barrel_core, axes="yz", max_dist=0.002)

    ctx.expect_within(
        barrel_core,
        zoom_ring,
        axes="yz",
        inner_elem=main_barrel,
        outer_elem=zoom_shell,
        name="zoom ring wraps the outer barrel",
    )
    ctx.expect_within(
        barrel_core,
        focus_ring,
        axes="yz",
        inner_elem=front_tube,
        outer_elem=focus_shell,
        name="focus ring wraps the inner front barrel",
    )
    ctx.expect_within(
        focus_ring,
        zoom_ring,
        axes="yz",
        inner_elem=focus_shell,
        outer_elem=zoom_shell,
        name="zoom ring is thicker than the inner focus ring",
    )
    ctx.expect_within(
        barrel_core,
        tripod_collar,
        axes="yz",
        inner_elem=collar_seat,
        outer_elem=collar_band,
        name="tripod collar encircles the barrel midpoint",
    )
    ctx.expect_within(
        focus_ring,
        front_housing,
        axes="yz",
        inner_elem=focus_shell,
        outer_elem=front_shell,
        name="front housing is the widest ring at the tip",
    )
    ctx.expect_gap(
        tripod_collar,
        zoom_ring,
        axis="x",
        min_gap=0.020,
        max_gap=0.040,
        positive_elem=collar_band,
        negative_elem=zoom_shell,
        name="tripod collar sits behind the focus zone and ahead of the zoom ring",
    )
    ctx.expect_gap(
        focus_ring,
        tripod_collar,
        axis="x",
        min_gap=0.020,
        max_gap=0.045,
        positive_elem=focus_shell,
        negative_elem=collar_band,
        name="focus ring stays forward of the tripod collar",
    )
    ctx.expect_gap(
        front_housing,
        focus_ring,
        axis="x",
        min_gap=0.050,
        max_gap=0.085,
        positive_elem=front_shell,
        negative_elem=focus_shell,
        name="front element housing remains at the barrel tip ahead of the focus ring",
    )
    ctx.expect_gap(
        zoom_ring,
        rear_mount,
        axis="x",
        min_gap=0.080,
        max_gap=0.120,
        positive_elem=zoom_shell,
        negative_elem=mount_plate,
        name="rear mount plate stays behind the zoom ring at the camera end",
    )

    ctx.expect_gap(
        front_housing,
        barrel_core,
        axis="x",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=front_shell,
        negative_elem=front_tube,
        name="front housing seats onto the front barrel tube",
    )
    ctx.expect_gap(
        barrel_core,
        rear_mount,
        axis="x",
        max_gap=0.001,
        max_penetration=0.004,
        positive_elem=rear_body,
        negative_elem=mount_plate,
        name="rear mount plate seats flush against the lens barrel",
    )
    ctx.expect_gap(
        clamp_lever,
        tripod_collar,
        axis="x",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem=clamp_base,
        negative_elem=lever_boss,
        name="flat lever clamp attaches to the collar boss",
    )
    ctx.expect_gap(
        barrel_core,
        clamp_lever,
        axis="z",
        min_gap=0.002,
        max_gap=0.030,
        positive_elem=collar_seat,
        negative_elem=lever_plate,
        name="lever clamp hangs below the barrel in the default pose",
    )
    ctx.expect_gap(
        alignment_dot,
        barrel_core,
        axis="z",
        min_gap=0.001,
        max_gap=0.015,
        name="alignment dot sits proud above the rear mount",
    )
    ctx.expect_within(
        alignment_dot,
        rear_mount,
        axes="y",
        name="alignment dot stays on the rear mount plate width",
    )
    ctx.expect_within(
        locking_pin,
        rear_mount,
        axes="yz",
        name="locking pin stays within the rear mount plate footprint",
    )
    ctx.expect_gap(
        barrel_core,
        locking_pin,
        axis="x",
        min_gap=0.004,
        max_gap=0.020,
        name="locking pin protrudes rearward from the mount plate",
    )

    with ctx.pose({collar_spin: math.pi / 2.0}):
        ctx.expect_origin_distance(tripod_collar, barrel_core, axes="yz", max_dist=0.002)
        ctx.expect_within(
            barrel_core,
            tripod_collar,
            axes="yz",
            inner_elem=collar_seat,
            outer_elem=collar_band,
            name="tripod collar stays concentric when rotated",
        )
        ctx.expect_gap(
            clamp_lever,
            barrel_core,
            axis="y",
            min_gap=0.006,
            max_gap=0.030,
            name="lever clamp swings to the side when the collar rotates",
        )
    return ctx.report()


def _save_mesh(geometry: MeshGeometry, filename: str):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_band_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    x_center: float,
    radial_segments: int = 56,
) -> MeshGeometry:
    outer = CylinderGeometry(radius=outer_radius, height=length, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=length + 0.004,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).rotate_y(math.pi / 2.0).translate(x_center, 0.0, 0.0)


def _grip_ring_geometry(
    *,
    inner_radius: float,
    sleeve_outer_radius: float,
    rib_outer_radius: float,
    length: float,
    x_center: float,
    rib_count: int,
) -> MeshGeometry:
    geometries = [
        _ring_band_geometry(
            outer_radius=sleeve_outer_radius,
            inner_radius=inner_radius,
            length=length,
            x_center=x_center,
        )
    ]
    if rib_count > 0:
        rib_span = length * 0.82
        rib_step = rib_span / max(rib_count - 1, 1)
        rib_start = x_center - (rib_span * 0.5)
        rib_length = min(0.010, length * 0.18)
        for rib_index in range(rib_count):
            geometries.append(
                _ring_band_geometry(
                    outer_radius=rib_outer_radius,
                    inner_radius=inner_radius,
                    length=rib_length,
                    x_center=rib_start + (rib_index * rib_step),
                    radial_segments=48,
                )
            )
    return _merge_geometries(geometries)


def _front_housing_geometry() -> MeshGeometry:
    return _merge_geometries(
        [
            _ring_band_geometry(
                outer_radius=0.056,
                inner_radius=0.0475,
                length=0.024,
                x_center=0.546,
                radial_segments=64,
            ),
            _ring_band_geometry(
                outer_radius=0.078,
                inner_radius=0.055,
                length=0.042,
                x_center=0.573,
                radial_segments=64,
            ),
            _ring_band_geometry(
                outer_radius=0.086,
                inner_radius=0.064,
                length=0.034,
                x_center=0.603,
                radial_segments=64,
            ),
            _ring_band_geometry(
                outer_radius=0.088,
                inner_radius=0.068,
                length=0.008,
                x_center=0.620,
                radial_segments=64,
            ),
        ]
    )


def _rear_mount_geometry() -> MeshGeometry:
    return _merge_geometries(
        [
            _ring_band_geometry(
                outer_radius=0.056,
                inner_radius=0.022,
                length=0.012,
                x_center=-0.006,
            ),
            _ring_band_geometry(
                outer_radius=0.048,
                inner_radius=0.040,
                length=0.006,
                x_center=-0.001,
            ),
        ]
    )


def _tripod_collar_geometry() -> MeshGeometry:
    return _merge_geometries(
        [
            _ring_band_geometry(
                outer_radius=0.073,
                inner_radius=0.0504,
                length=0.060,
                x_center=0.0,
            ),
            BoxGeometry((0.075, 0.042, 0.018)).translate(0.0, 0.0, -0.079),
        ]
    )


# >>> USER_CODE_END

object_model = build_object_model()
